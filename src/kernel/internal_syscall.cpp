#include "internal_syscall.h"
#include "constants.h"
#include "debug.h"
#include "gic.h"
#include "idle_manager.h"
#include "kernel_state.h"
#include "mcp2515.h"
#include "message.h"
#include "rpi.h"
#include "syscall.h"
#include "task_descriptor.h"
#include "time.h"
#include "uart.h"
#include <cstdint>
#include <cstring>

uint64_t get_cycle_count() {
  uint64_t cycle_count;
  asm volatile("mrs %0, cntvct_el0" : "=r"(cycle_count));
  return cycle_count;
}

extern "C" void default_handler(int n) {
  uint64_t esr_el1;
  uint64_t far_el1;
  uint64_t elr_el1;
  uint64_t spsr_el1;

  asm volatile("mrs %0, esr_el1" : "=r"(esr_el1));
  asm volatile("mrs %0, far_el1" : "=r"(far_el1));
  asm volatile("mrs %0, elr_el1" : "=r"(elr_el1));
  asm volatile("mrs %0, spsr_el1" : "=r"(spsr_el1));

  debug_printf(
      CONSOLE, "DEFAULT VBAR HANDLER %u HIT ESR=%x FAR=%x ELR=%x SPSR=%x\n\r",
      n, static_cast<unsigned int>(esr_el1), static_cast<unsigned int>(far_el1),
      static_cast<unsigned int>(elr_el1), static_cast<unsigned int>(spsr_el1));
}

// Allocates a new task, initalizes descriptor and stack

// Wake the TX notifier only when FR says we can send (!TXFF).
// If not ready, mask the firing source without waking.
// GIC 153 is shared across PL011s; demux with PACTL_CS then per-UART MIS.
static void handle_uart_tx(size_t line, Event event, uint32_t pactl_bit,
                           uint32_t pactl) {
  if (!(pactl & pactl_bit)) {
    return;
  }
  if (!is_uart_mis_tx_pending(line)) {
    return;
  }
  if (can_transmit_io(line)) {
    kernel_runtime.event_controller.handle_event(event);
    return;
  }
  disable_uart_interrupt(UARTInterruptType::TXIM, line);
}

// static void handle_uart_irq() {
//   const uint32_t pactl = read_pactl_cs();

//   if ((pactl & PACTL_UART0_IRQ) && is_uart_mis_rx_pending(CONSOLE)) {
//     kernel_runtime.event_controller.handle_event(Event::UART_RX_IRQ);
//   }

//   handle_uart_tx(CONSOLE, Event::UART_TX_IRQ, PACTL_UART0_IRQ, pactl);
//   handle_uart_tx(WEBSERIAL, Event::UART3_TX_IRQ, PACTL_UART3_IRQ, pactl);
// }

static void handle_uart_irq() {
  if (is_uart_mis_rx_pending(CONSOLE)) {
    kernel_runtime.event_controller.handle_event(Event::UART_RX_IRQ);
  }

  // Shared UART IRQ line can be noisy / hard to source-demux in emulation.
  // Broadcast TX wakeups; spurious notifier wakeups are acceptable.
  kernel_runtime.event_controller.handle_event(Event::UART_TX_IRQ);
  kernel_runtime.event_controller.handle_event(Event::UART3_TX_IRQ);
}

static void handle_mcp2515_irq() {
  auto source = mcp2515_get_active_irq();
  if (!gpio_get_event_detect_status(17)) {
    debug_printf(CONSOLE, "GPIO 17 event detect not set\n\r");
    return;
  }
  disable_mcp2515_interrupt(source);
  if (source.rxi0ie || source.rxi1e) {
    kernel_runtime.event_controller.handle_event(Event::CAN_RX_IRQ);
  } else if (source.tx0ie || source.tx1ie || source.tx2ie) {
    kernel_runtime.event_controller.handle_event(Event::CAN_TX_IRQ);
  } else {
    debug_printf(CONSOLE, "Unhandled MCP2515 IRQ %d\n\r", source);
  }

  gpio_clr_event_detect_status(17);
}

static void handle_interrupt() {
  // choose next task to run
  // restore chosen task context, return from exeption with eret
  // loop through all pending interrupts
  for (;;) {
    uint32_t gic_iar      = gic_iar_read();
    uint32_t interrupt_id = gic_iar & GIC_IAR_ID_MASK;

    // break if no more interrupts
    if (interrupt_id == GIC_SPURIOUS_IRQ) {
      return;
    }

    // interrupt id to event mapping
    switch (interrupt_id) {
    case GIC_TIMER_IRQ_C1:
      kernel_runtime.event_controller.handle_event(Event::CLOCK_TICK);
      break;
    case GIC_TIMER_IRQ_C3:
      debug_printf(CONSOLE, "5 second delay event\n\r");
      kernel_runtime.event_controller.handle_event(Event::DELAY_5S);
      break;
    case GIC_UART_IRQ:
      handle_uart_irq();
      break;
    case GIC_MCP2515_IRQ: {
      handle_mcp2515_irq();
      break;
    }
    default:
      break;
    }

    gic_eoi(gic_iar);
  }
}

Syscall activate_task(TaskDescriptor &td) {
  td.state = TaskStatus::RUNNING;

  // clear I and F bits in saved Pstate to allow interrupts in user mode.
  auto *user_tf         = reinterpret_cast<TrapFrame *>(td.sp_el0);
  uint64_t pstate_mask  = 0x3 << 6;
  user_tf->spsr_el1    &= ~pstate_mask;

  // switch to user mode
  // this will return when task makes a syscall
  TrapFrame *tf = _switch_to_user(td.sp_el0);
  td.sp_el0     = reinterpret_cast<uint64_t>(tf);

  // check if interrupt
  if (tf->is_interrupt) {
    handle_interrupt();
    return Syscall::YIELD;
  }

  // https://developer.arm.com/documentation/ddi0595/2020-12/AArch64-Registers/ESR-EL1--Exception-Syndrome-Register--EL1-
  Syscall svc_imm = static_cast<Syscall>(tf->esr_el1 & 0xFFFF);

  // this is for us to decide how to encode the syscall
  // for now we are just reading x0
  return svc_imm;
}

void handle(TaskId tid, Syscall request) {
  // this will handle the given request code and perform the appropriate action
  // (e.g. for syscalls) ESR_EL1 will have exception code, holds n form svc N

  auto td = kernel_runtime.task_table.lookup_td(tid);
  if (td == nullptr) {
    panic("invalid tid");
  }
  TrapFrame *tf = reinterpret_cast<TrapFrame *>(td->sp_el0);

  switch (request) {
  case Syscall::CREATE: {
    int priority       = tf->x[0];
    void (*function)() = reinterpret_cast<void (*)()>(tf->x[1]);
    TaskId new_tid =
        kernel_runtime.task_table.create_task(priority, function, tid);
    tf->x[0] = new_tid;
    kernel_runtime.scheduler.schedule(new_tid, priority);
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::MY_TID: {
    tf->x[0] = tid;
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::MY_PARENT_TID: {
    tf->x[0] = td->parent_tid;
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::YIELD: {
    td->state = TaskStatus::READY;
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::EXIT: {
    td->state = TaskStatus::TERMINATED;

    TaskExitMsg msg{};

    // wake sender queue to alert of task exist
    auto to_tid_opt = td->sender_queue.pop();
    while (to_tid_opt.has_value()) {
      auto to_tid = to_tid_opt.value();
      auto to_td  = kernel_runtime.task_table.lookup_td(to_tid);
      _assert(to_td != nullptr, "invalid tid in sender queue");
      if (to_td == nullptr) {
        to_tid_opt = td->sender_queue.pop();
        continue;
      }
      auto to_tf = reinterpret_cast<TrapFrame *>(to_td->sp_el0);

      char *rcv_reply = reinterpret_cast<char *>(to_tf->x[3]);
      int rcv_len     = to_tf->x[4];

      std::memcpy(rcv_reply, &msg, rcv_len);
      to_td->state = TaskStatus::READY;
      kernel_runtime.scheduler.schedule(*to_td);
      to_tid_opt = td->sender_queue.pop();
    }

    kernel_runtime.task_table.delete_task(tid);
    kernel_runtime.event_controller.handle_event(Event::TASK_EXIT, tid);
    break;
  }
  case Syscall::SEND: {
    int to_tid = tf->x[0];

    auto to_td = kernel_runtime.task_table.lookup_td(to_tid);
    if (to_td == nullptr) {
      tf->x[0] = -1; // invalid tid
      kernel_runtime.scheduler.schedule(*td);
      break;
    }
    if (to_tid == tid) {
      tf->x[0] = -2; // can't send to self
      kernel_runtime.scheduler.schedule(*td);
      break;
    }

    if (to_td->state == TaskStatus::W4_SEND) {
      auto to_tf = reinterpret_cast<TrapFrame *>(to_td->sp_el0);

      // set the sender tid (x0 is a pointer to a int)
      *reinterpret_cast<int *>(to_tf->x[0]) = tid;

      // overwrite x0 to return value of message length
      int msg_len = tf->x[2];
      int rcv_len = to_tf->x[2];
      int len = to_tf->x[0] = std::min(msg_len, rcv_len);

      // copy message from sender to receiver
      const char *msg = reinterpret_cast<const char *>(tf->x[1]);
      char *rcv_buf   = reinterpret_cast<char *>(to_tf->x[1]);
      std::memcpy(rcv_buf, msg, len);

      // skip the W4_RECEIVE state, someone was already waiting
      td->state    = TaskStatus::W4_REPLY;
      to_td->state = TaskStatus::READY;
      kernel_runtime.scheduler.schedule(*to_td);
    } else {
      td->state = TaskStatus::W4_RECEIVE;
      to_td->sender_queue.push(tid);
    }

    break;
  }
  case Syscall::RECEIVE: {
    auto from_tid_opt = td->sender_queue.pop();
    if (from_tid_opt.has_value()) {
      int from_tid = from_tid_opt.value();

      auto to_td = kernel_runtime.task_table.lookup_td(from_tid);
      _assert(to_td != nullptr, "invalid tid");
      auto from_tf = reinterpret_cast<TrapFrame *>(to_td->sp_el0);

      // set who msg is from (follow int ptr)
      *reinterpret_cast<int *>(tf->x[0]) = from_tid;

      // set msg length (overwrite x0 / arg0)
      int msg_len = from_tf->x[2];
      int rcv_len = tf->x[2];
      int len = tf->x[0] = std::min(msg_len, rcv_len);

      // copy over buffer
      const char *msg = reinterpret_cast<const char *>(from_tf->x[1]);
      char *rcv_buf   = reinterpret_cast<char *>(tf->x[1]);
      std::memcpy(rcv_buf, msg, len);

      // update sender task to waiting for reply
      // however no impact on scheduling
      to_td->state = TaskStatus::W4_REPLY;
      td->state    = TaskStatus::READY;
      kernel_runtime.scheduler.schedule(*td);
    } else {
      td->state = TaskStatus::W4_SEND;
    }
    break;
  }
  case Syscall::REPLY: {
    int to_tid = tf->x[0];

    auto to_td = kernel_runtime.task_table.lookup_td(to_tid);
    if (to_td == nullptr) {
      tf->x[0] = -1; // invalid tid
      _assert(false, "invalid tid");
      kernel_runtime.scheduler.schedule(*td);
      break;
    }

    if (to_td->state != TaskStatus::W4_REPLY) {
      tf->x[0] = -2; // task not waiting for reply
      kernel_runtime.scheduler.schedule(*td);
      break;
    }

    auto to_tf        = reinterpret_cast<TrapFrame *>(to_td->sp_el0);
    const char *reply = reinterpret_cast<const char *>(tf->x[1]);
    int reply_len     = tf->x[2];

    char *rcv_reply = reinterpret_cast<char *>(to_tf->x[3]);
    int rcv_len     = to_tf->x[4];
    int len = to_tf->x[0] = tf->x[0] = std::min(reply_len, rcv_len);
    std::memcpy(rcv_reply, reply, len);

    to_td->state = TaskStatus::READY;
    kernel_runtime.scheduler.schedule(*to_td);
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::AWAIT_EVENT: {
    auto raw_event = static_cast<int>(tf->x[0]);
    if (raw_event < 0 || raw_event >= static_cast<int>(Event::EVENT_COUNT)) {
      tf->x[0] = -1;
      kernel_runtime.scheduler.schedule(*td);
      break;
    }
    auto event = static_cast<Event>(raw_event);
    kernel_runtime.event_controller.await_event(event, tid);
    break;
  }
  case Syscall::EMIT_EVENT: {
    auto raw_event = static_cast<int>(tf->x[0]);
    if (raw_event < 0 || raw_event >= static_cast<int>(Event::EVENT_COUNT)) {
      tf->x[0] = -1;
      kernel_runtime.scheduler.schedule(*td);
      break;
    }
    kernel_runtime.event_controller.handle_event(static_cast<Event>(raw_event));
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::PARK: {
    kernel_runtime.idle_manager.go_idle();
    td->state = TaskStatus::READY;
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::KERNEL_IDLE_PCT: {
    tf->x[0] = kernel_runtime.idle_manager.get_idle_time_percentage();
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::TX_CAN: {
    const CANFRAME *frame_ptr = reinterpret_cast<const CANFRAME *>(tf->x[0]);
    CANFRAME frame            = *frame_ptr;
    tf->x[0]                  = mcp2515_send(frame);
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  case Syscall::RX_CAN: {
    CANFRAME *frame_ptr = reinterpret_cast<CANFRAME *>(tf->x[0]);
    tf->x[0]            = mcp2515_recieve(*frame_ptr);
    kernel_runtime.scheduler.schedule(*td);
    break;
  }
  }
  return;
}
