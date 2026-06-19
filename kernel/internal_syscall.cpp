#include "internal_syscall.h"
#include "debug.h"
#include "gic.h"
#include "idle_manager.h"
#include "kernel_state.h"
#include "map.h"
#include "mcp2515.h"
#include "message.h"
#include "rpi.h"
#include "scheduler.h"
#include "syscall.h"
#include "task_descriptor.h"
#include "time.h"
#include "uart.h"
#include <cstdint>
#include <optional>

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

  debug_printf(CONSOLE,
               "DEFAULT VBAR HANDLER %u HIT ESR=%x FAR=%x ELR=%x SPSR=%x\n\r",
               n, (unsigned int)esr_el1, (unsigned int)far_el1,
               (unsigned int)elr_el1, (unsigned int)spsr_el1);
}

extern "C" void task_entry_wrapper(void (*function)()) {
  function();
  exit();
}

// Allocates a new task, initalizes descriptor and stack
int _create(int priority, void (*function)(), int parent_tid) {
  using namespace Kernel;

  if (0 > priority || priority >= PRIORITY_LEVELS) {
    return -1; // invalid priority
  }

  auto descriptor_index_opt = task_allocator.allocate();
  if (descriptor_index_opt == std::nullopt) {
    _assert(false, "No free task descriptors");
    return -2; // no free task descriptors
  }
  auto td_idx = descriptor_index_opt.value();
  auto tid    = next_tid++;

  // define task stack (grows downwards)
  uint64_t task_stack_base = (uint64_t)&task_stacks[td_idx][TASK_STACK_SIZE];
  uint64_t task_stack_end  = task_stack_base - TASK_STACK_SIZE;

  // clear stack memory (not required but helpful)
  __builtin_memset((void *)(task_stack_end), 0, TASK_STACK_SIZE);

  // build & push inital trapframe
  TrapFrame *tf    = (TrapFrame *)(task_stack_base - sizeof(TrapFrame));
  tf->elr_el1      = (uint64_t)task_entry_wrapper;
  tf->x[0]         = (uint64_t)function;
  tf->spsr_el1     = 0;
  tf->is_interrupt = 0;

  auto &td = task_descriptors[td_idx] = {.td_idx     = td_idx,
                                         .tid        = tid,
                                         .parent_tid = parent_tid,
                                         .priority   = priority,
                                         .state      = TaskStatus::READY,
                                         .sp_el0     = (uint64_t)tf};

  tid_to_descriptor.set(tid, td_idx);
  Kernel::scheduler.schedule(td);
  return tid;
}

static void uninitialize_event(Event event) {
  using namespace Kernel;
  initalized_events &= ~(1u << static_cast<int>(event));
}

static void handle_event(Event event);

static void initalize_event(Event event) {
  using namespace Kernel;

  // check if we've already initalized this event
  if (initalized_events & (1u << static_cast<int>(event)))
    return;
  initalized_events |= (1u << static_cast<int>(event));

  switch (event) {
  case Event::CLOCK_TICK_1MS: {
    set_interrupt_core_routing(0, GIC_TIMER_IRQ_C1, true);
    set_interrupt(GIC_TIMER_IRQ_C1, true);
    clear_timer_interrupt(1);
    set_timer_interrupt(1, TIME_10MS_US);
    break;
  }
  case Event::DELAY_5S: {
    set_interrupt_core_routing(0, GIC_TIMER_IRQ_C3, true);
    set_interrupt(GIC_TIMER_IRQ_C3, true);
    clear_timer_interrupt(3);
    set_timer_interrupt(3, TIME_1S_US * 5);
    break;
  }
  case Event::UART_RX_IRQ: {
    // unmask rtim and rxim
    enable_uart_interrupt(UARTInterruptType::RTIM);
    enable_uart_interrupt(UARTInterruptType::RXIM);
    break;
  }
  case Event::UART_TX_IRQ: {
    enable_uart_interrupt(UARTInterruptType::TXIM);
    enable_uart_interrupt(UARTInterruptType::CTSMIM);
    break;
  }
  case Event::CAN_RX_IRQ: {
    auto active = mcp2515_get_active_irq();
    if (active.rxi0ie || active.rxi1e) {
      // short circuit, handle them if they're already active
      handle_event(Event::CAN_RX_IRQ);
    } else {
      // set up interrupts
      set_interrupt_core_routing(0, GIC_MCP2515_IRQ, true);
      set_interrupt(GIC_MCP2515_IRQ, true);
      clear_mcp2515_interrupt(CANINT{.rxi1e = true, .rxi0ie = true});
      enable_mcp2515_interrupt(CANINT{.rxi1e = true, .rxi0ie = true});
    }
    break;
  }
  case Event::CAN_TX_IRQ: {
    auto active = mcp2515_get_active_irq();
    if (active.tx0ie || active.tx1ie || active.tx2ie) {
      // short circuit, handle them if they're already active
      handle_event(Event::CAN_TX_IRQ);
    } else {
      // set up interrupts
      set_interrupt_core_routing(0, GIC_MCP2515_IRQ, true);
      set_interrupt(GIC_MCP2515_IRQ, true);
      clear_mcp2515_interrupt(CANINT{
          .tx2ie = true,
          .tx1ie = true,
          .tx0ie = true,
      });
      enable_mcp2515_interrupt(CANINT{
          .tx2ie = true,
          .tx1ie = true,
          .tx0ie = true,
      });
    }
    break;
  }
  default: {
    break;
  }
  }
}

static void handle_event(Event event) {
  using namespace Kernel;

  // one-time handling
  switch (event) {
  case Event::CLOCK_TICK_1MS: {
    update_timer_interrupt(1, TIME_10MS_US);
    clear_timer_interrupt(1);
    break;
  }
  case Event::DELAY_5S: {
    clear_timer_interrupt(3);
    uninitialize_event(event);
    break;
  }
  case Event::UART_RX_IRQ: {
    // mask so no RX IRQ fires until notifier re-await_event
    disable_uart_interrupt(UARTInterruptType::RXIM);
    disable_uart_interrupt(UARTInterruptType::RTIM);
    uninitialize_event(event);
    break;
  }
  case Event::UART_TX_IRQ: {
    // immediately disable after firing as they will keep firing
    disable_uart_interrupt(UARTInterruptType::TXIM);
    disable_uart_interrupt(UARTInterruptType::CTSMIM);
    uninitialize_event(event);
    break;
  }
  case Event::CAN_RX_IRQ: {
    uninitialize_event(event);
    break;
  }
  case Event::CAN_TX_IRQ: {
    uninitialize_event(event);
    break;
  }
  default: {
    break;
  }
  }

  if (!event_buffers.contains(event)) {
    _assert(false, "Received event with no waiting tasks");
    return;
  }

  // handle buffer of waiting tasks
  // some events may require all tasks to wake
  // some events may require the first to wake
  auto event_buf = event_buffers.get_ref(event);
  auto tid_opt   = event_buf->pop();
  while (tid_opt.has_value()) {
    auto tid = tid_opt.value();
    auto td  = lookup_td(tid);
    if (!td.has_value()) {
      tid_opt = event_buf->pop();
      continue;
    }

    switch (event) {
    case Event::DELAY_5S: {
      scheduler.schedule(*td.value());

      // reset the delay for the next task.
      if (!event_buf->is_empty()) {
        initalize_event(event);
      }
      return; // return if only the first should wake
    }
    case Event::CAN_RX_IRQ: {
      scheduler.schedule(*td.value());
      return;
    }
    default: {
      scheduler.schedule(*td.value());
      break;
    }
    }

    tid_opt = event_buf->pop();
  }
}

// Wake the TX notifier only when FR says we can send (!TXFF and CTS up).
// If not ready, mask/clear the firing source without waking; the other
// interrupt (still armed from await_event) covers the remaining condition.
static void handle_uart_irq() {
  // if rx is a cause of interrupt
  if (is_uart_mis_rx_pending()) {
    handle_event(Event::UART_RX_IRQ);
  }

  // handling tx interrupts
  if (!is_uart_mis_tx_pending() && !is_uart_mis_cts_pending()) {
    return;
  }

  if (can_transmit_io()) {
    handle_event(Event::UART_TX_IRQ);
    return;
  }

  if (is_uart_mis_tx_pending()) {
    disable_uart_interrupt(UARTInterruptType::TXIM);
  }
  if (is_uart_mis_cts_pending()) {
    clear_uart_interrupt(UARTInterruptType::CTSMIM);
  }
}

static void handle_mcp2515_irq() {
  auto source = mcp2515_get_irq_source();
  if (!gpio_get_event_detect_status(17)) {
    debug_printf(CONSOLE, "GPIO 17 event detect not set\n\r");
    return;
  }
  disable_mcp2515_interrupt(source);
  if (source.rxi0ie || source.rxi1e) {
    handle_event(Event::CAN_RX_IRQ);
  } else if (source.tx0ie || source.tx1ie || source.tx2ie) {
    handle_event(Event::CAN_TX_IRQ);
  } else {
    debug_printf(CONSOLE, "Unhandled MCP2515 IRQ\n\r");
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
      handle_event(Event::CLOCK_TICK_1MS);
      break;
    case GIC_TIMER_IRQ_C3:
      debug_printf(CONSOLE, "5 second delay event\n\r");
      handle_event(Event::DELAY_5S);
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

Syscall activate(int tid) {
  auto td_opt = Kernel::lookup_td(tid);
  auto td     = td_opt.value();
  td->state   = TaskStatus::RUNNING;

  // clear I and F bits in saved Pstate to allow interrupts in user mode.
  auto *user_tf         = (Kernel::TrapFrame *)td->sp_el0;
  uint64_t pstate_mask  = 0x3 << 6;
  user_tf->spsr_el1    &= ~pstate_mask;

  // switch to user mode
  // this will return when task makes a syscall
  Kernel::TrapFrame *tf = _switch_to_user(td->sp_el0);
  td->sp_el0            = (uint64_t)tf;

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

void handle(int tid, Syscall request) {
  // this will handle the given request code and perform the appropriate action
  // (e.g. for syscalls) ESR_EL1 will have exception code, holds n form svc N

  using namespace Kernel;
  auto td_opt   = lookup_td(tid);
  auto td       = td_opt.value();
  TrapFrame *tf = (TrapFrame *)td->sp_el0;

  uint64_t start_cycle = get_cycle_count();
  switch (request) {
  case Syscall::CREATE: {
    int new_tid = _create(tf->x[0], (void (*)())tf->x[1], tid);
    tf->x[0]    = new_tid;
    scheduler.schedule(*td);
    break;
  }
  case Syscall::MY_TID: {
    tf->x[0] = tid;
    scheduler.schedule(*td);
    break;
  }
  case Syscall::MY_PARENT_TID: {
    tf->x[0] = td->parent_tid;
    scheduler.schedule(*td);
    break;
  }
  case Syscall::YIELD: {
    td->state = TaskStatus::READY;
    scheduler.schedule(*td);
    break;
  }
  case Syscall::EXIT: {
    td->state = TaskStatus::TERMINATED;
    tid_to_descriptor.remove(tid);
    task_allocator.free(td->td_idx);

    TaskExitMsg msg{};

    // wake sender queue to alert of task exist
    auto to_tid_opt = td->sender_queue.pop();
    while (to_tid_opt.has_value()) {
      auto to_tid    = to_tid_opt.value();
      auto to_td_opt = lookup_td(to_tid);
      if (!to_td_opt.has_value()) {
        to_tid_opt = td->sender_queue.pop();
        continue;
      }
      auto to_td = to_td_opt.value();
      auto to_tf = (TrapFrame *)to_td->sp_el0;

      char *rcv_reply = (char *)to_tf->x[3];
      int rcv_len     = to_tf->x[4];

      __builtin_memcpy(rcv_reply, &msg, rcv_len);
      to_td->state = TaskStatus::READY;
      scheduler.schedule(*to_td);
      to_tid_opt = td->sender_queue.pop();
    }
    break;
  }
  case Syscall::SEND: {
    int to_tid = tf->x[0];

    auto to_td_opt = lookup_td(to_tid);
    if (!to_td_opt.has_value()) {
      tf->x[0] = -1; // invalid tid
      scheduler.schedule(*td);
      break;
    }

    auto to_td = to_td_opt.value();
    if (to_tid == tid) {
      tf->x[0] = -2; // can't send to self
      scheduler.schedule(*td);
      break;
    }

    if (to_td->state == TaskStatus::W4_SEND) {
      auto to_tf = (TrapFrame *)to_td->sp_el0;

      // set the sender tid (x0 is a pointer to a int)
      *(int *)to_tf->x[0] = tid;

      // overwrite x0 to return value of message length
      int msg_len = tf->x[2];
      int rcv_len = to_tf->x[2];
      int len = to_tf->x[0] = std::min(msg_len, rcv_len);

      // copy message from sender to receiver
      const char *msg = (const char *)tf->x[1];
      char *rcv_buf   = (char *)to_tf->x[1];
      __builtin_memcpy(rcv_buf, msg, len);

      // skip the W4_RECEIVE state, someone was already waiting
      td->state    = TaskStatus::W4_REPLY;
      to_td->state = TaskStatus::READY;
      scheduler.schedule(*to_td);
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

      auto to_td_opt = lookup_td(from_tid);
      auto to_td     = to_td_opt.value();
      auto from_tf   = (TrapFrame *)to_td->sp_el0;

      // set who msg is from (follow int ptr)
      *(int *)tf->x[0] = from_tid;

      // set msg length (overwrite x0 / arg0)
      int msg_len = from_tf->x[2];
      int rcv_len = tf->x[2];
      int len = tf->x[0] = std::min(msg_len, rcv_len);

      // copy over buffer
      const char *msg = (const char *)from_tf->x[1];
      char *rcv_buf   = (char *)tf->x[1];
      __builtin_memcpy(rcv_buf, msg, len);

      // update sender task to waiting for reply
      // however no impact on scheduling
      to_td->state = TaskStatus::W4_REPLY;
      td->state    = TaskStatus::READY;
      scheduler.schedule(*td);
    } else {
      td->state = TaskStatus::W4_SEND;
    }
    break;
  }
  case Syscall::REPLY: {
    int to_tid = tf->x[0];

    auto to_td_opt = lookup_td(to_tid);
    if (!to_td_opt.has_value()) {
      tf->x[0] = -1; // invalid tid
      scheduler.schedule(*td);
      break;
    }
    auto to_td = to_td_opt.value();

    if (to_td->state != TaskStatus::W4_REPLY) {
      tf->x[0] = -2; // task not waiting for reply
      scheduler.schedule(*td);
      break;
    }

    auto to_tf        = (TrapFrame *)to_td->sp_el0;
    const char *reply = (const char *)tf->x[1];
    int reply_len     = tf->x[2];

    char *rcv_reply = (char *)to_tf->x[3];
    int rcv_len     = to_tf->x[4];
    int len = to_tf->x[0] = tf->x[0] = std::min(reply_len, rcv_len);
    __builtin_memcpy(rcv_reply, reply, len);

    to_td->state = TaskStatus::READY;
    scheduler.schedule(*to_td);
    scheduler.schedule(*td);
    break;
  }
  case Syscall::AWAIT_EVENT: {
    auto raw_event = static_cast<int>(tf->x[0]);
    if (raw_event < 0 || raw_event > static_cast<int>(Event::EVENT_COUNT)) {
      tf->x[0] = -1;
      scheduler.schedule(*td);
      break;
    }

    auto event = static_cast<Event>(raw_event);
    if (!event_buffers.contains(event)) {
      event_buffers.set(event, {});
    }
    auto event_buf = event_buffers.get_ref(event);
    event_buf->push(tid);
    initalize_event(event);
    break;
  }
  case Syscall::PARK: {
    idle_manager.go_idle();
    td->state = TaskStatus::READY;
    scheduler.schedule(*td);
    break;
  }
  case Syscall::KERNEL_IDLE_PCT: {
    tf->x[0] = idle_manager.get_idle_time_percentage();
    scheduler.schedule(*td);
    break;
  }
  case Syscall::TX_CAN: {
    const CANFRAME &frame = (const CANFRAME &)tf->x[0];
    tf->x[0]              = mcp2515_send(frame);
    scheduler.schedule(*td);
    break;
  }
  case Syscall::RX_CAN: {
    CANFRAME &frame = (CANFRAME &)tf->x[0];
    tf->x[0]        = mcp2515_recieve(frame);
    scheduler.schedule(*td);
    break;
  }
  }
  uint64_t end_cycle = get_cycle_count();
  if (syscall_cycle_counts.contains(request)) {
    uint64_t total_cycles = syscall_cycle_totals.get(request).value();
    uint64_t count_cycles = syscall_cycle_counts.get(request).value();
    syscall_cycle_totals.set(request, total_cycles + 1);
    syscall_cycle_counts.set(request, count_cycles + (end_cycle - start_cycle));
  } else {
    syscall_cycle_totals.set(request, 1);
    syscall_cycle_counts.set(request, end_cycle - start_cycle);
  }
  return;
}
