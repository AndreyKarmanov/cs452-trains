#include <cstdint>
#include <optional>

#include "internal_syscall.h"
#include "kernel_state.h"
#include "message.h"
#include "scheduler.h"
#include "syscall.h"
#include "task_descriptor.h"
#include "uart.h"

extern "C" void default_handler(int n) {
  uint64_t esr_el1;
  uint64_t far_el1;
  uint64_t elr_el1;
  uint64_t spsr_el1;

  asm volatile("mrs %0, esr_el1" : "=r"(esr_el1));
  asm volatile("mrs %0, far_el1" : "=r"(far_el1));
  asm volatile("mrs %0, elr_el1" : "=r"(elr_el1));
  asm volatile("mrs %0, spsr_el1" : "=r"(spsr_el1));

  uart_printf(CONSOLE,
              "DEFAULT VBAR HANDLER %u HIT ESR=%x FAR=%x ELR=%x SPSR=%x\n\r", n,
              (unsigned int)esr_el1, (unsigned int)far_el1,
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
  _assert(descriptor_index_opt != std::nullopt, "No free task descriptors");
  if (descriptor_index_opt == std::nullopt) {
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
  TrapFrame *tf = (TrapFrame *)(task_stack_base - sizeof(TrapFrame));
  tf->elr_el1   = (uint64_t)task_entry_wrapper;
  tf->x[0]      = (uint64_t)function;
  tf->spsr_el1  = 0;

  task_descriptors[td_idx] = {.td_idx     = td_idx,
                              .tid        = tid,
                              .parent_tid = parent_tid,
                              .priority   = priority,
                              .state      = TaskStatus::READY,
                              .sp_el0     = (uint64_t)tf};

  _assert(tid_to_descriptor.set(tid, td_idx), "failed to register tid");
  return tid;
}

Syscall activate(int tid) {
  TaskDescriptor &td = Kernel::require_td(tid);
  td.state           = TaskStatus::RUNNING;

  // switch to user mode
  // this will return when task makes a syscall
  Kernel::TrapFrame *tf = _switch_to_user(td.sp_el0);
  td.sp_el0             = (uint64_t)tf;

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
  TaskDescriptor &td = require_td(tid);
  TrapFrame *tf      = (TrapFrame *)td.sp_el0;

  switch (request) {
  case Syscall::CREATE: {
    int new_tid = _create(tf->x[0], (void (*)())tf->x[1], tid);
    auto new_td = lookup_td(new_tid);
    if (new_td != nullptr) {
      scheduler.schedule(*new_td);
    }
    tf->x[0] = new_tid;
    scheduler.schedule(td);
    break;
  }
  case Syscall::MY_TID: {
    tf->x[0] = tid;
    scheduler.schedule(td);
    break;
  }
  case Syscall::MY_PARENT_TID: {
    tf->x[0] = td.parent_tid;
    scheduler.schedule(td);
    break;
  }
  case Syscall::YIELD: {
    td.state = TaskStatus::READY;
    scheduler.schedule(td);
    break;
  }
  case Syscall::EXIT: {
    td.state = TaskStatus::TERMINATED;
    tid_to_descriptor.remove(tid);
    task_allocator.free(td.td_idx);

    // wake sender queue to alert of task exist
    auto to_tid_opt = td.sender_queue.pop();
    while (to_tid_opt.has_value()) {
      auto to_tid = to_tid_opt.value();
      auto *to_td = lookup_td(to_tid);
      if (to_td == nullptr) {
        to_tid_opt = td.sender_queue.pop();
        continue;
      }

      auto to_tf = (TrapFrame *)to_td->sp_el0;

      Message msg{};
      msg.type = MessageType::TASK_EXIT;

      char *rcv_reply = (char *)to_tf->x[3];
      int rcv_len     = to_tf->x[4];

      __builtin_memcpy(rcv_reply, &msg, rcv_len);
      to_td->state = TaskStatus::READY;
      scheduler.schedule(*to_td);
      to_tid_opt = td.sender_queue.pop();
    }

    break;
  }
  case Syscall::SEND: {
    int to_tid = tf->x[0];

    auto *to_td = lookup_td(to_tid);
    if (to_td == nullptr) {
      tf->x[0] = -1; // invalid tid
      scheduler.schedule(td);
      break;
    }

    if (to_tid == tid) {
      tf->x[0] = -2; // can't send to self
      scheduler.schedule(td);
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
      td.state     = TaskStatus::W4_REPLY;
      to_td->state = TaskStatus::READY;
      scheduler.schedule(*to_td);
    } else {
      td.state = TaskStatus::W4_RECEIVE;
      to_td->sender_queue.push(tid);
    }

    break;
  }
  case Syscall::RECEIVE: {
    auto from_tid_opt = td.sender_queue.pop();
    if (from_tid_opt.has_value()) {
      int from_tid = from_tid_opt.value();

      auto *to_td = lookup_td(from_tid);
      _assert(to_td != nullptr, "sender tid missing from map");
      auto from_tf = (TrapFrame *)to_td->sp_el0;

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
      td.state     = TaskStatus::READY;
      scheduler.schedule(td);
    } else {
      td.state = TaskStatus::W4_SEND;
    }
    break;
  }
  case Syscall::REPLY: {
    int to_tid = tf->x[0];

    auto *to_td = lookup_td(to_tid);
    if (to_td == nullptr) {
      tf->x[0] = -1; // invalid tid
      scheduler.schedule(td);
      break;
    }

    if (to_td->state != TaskStatus::W4_REPLY) {
      tf->x[0] = -2; // task not waiting for reply
      scheduler.schedule(td);
      break;
    }

    auto to_tf        = (TrapFrame *)to_td->sp_el0;
    const char *reply = (const char *)tf->x[1];
    int reply_len     = tf->x[2];

    char *rcv_reply = (char *)to_tf->x[3];
    int rcv_len     = to_tf->x[4];
    int len = to_tf->x[0] = tf->x[0] = std::min(reply_len, rcv_len);
    __builtin_memcpy(rcv_reply, reply, len);

    _assert(to_td->state == TaskStatus::W4_REPLY,
            "TASK NOT WAITING FOR REPLY\r\n");

    to_td->state = TaskStatus::READY;
    scheduler.schedule(*to_td);
    scheduler.schedule(td);
    break;
  }
  }
  return;
}
