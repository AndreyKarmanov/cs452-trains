#include <cstdint>
#include <optional>

#include "internal_syscall.h"
#include "kernel_state.h"
#include "scheduler.h"
#include "syscall.h"
#include "task_descriptor.h"
#include "uart.h"

extern "C" void default_handler() {
  uart_puts(CONSOLE, "DEFAULT VBAR HANDLER HIT\n\r");
}

// Allocates a new task, initalizes descriptor and stack
int _create(int priority, void (*function)()) {
  using namespace Kernel;

  if (0 > priority || priority >= PRIORITY_LEVELS) {
    return -1; // invalid priority
  }

  auto tid_opt = task_allocator.allocate();
  _assert(tid_opt != std::nullopt, "No free task descriptors");
  if (tid_opt == std::nullopt) {
    return -2; // no free task descriptors
  }
  auto tid = tid_opt.value();

  // define task stack (grows downwards)
  uint64_t task_stack_base = (uint64_t)&task_stacks[tid][TASK_STACK_SIZE];
  uint64_t task_stack_end  = task_stack_base - TASK_STACK_SIZE;

  // clear stack memory (not required but helpful)
  __builtin_memset((void *)(task_stack_end), 0, TASK_STACK_SIZE);

  // build & push inital trapframe
  TrapFrame *tf = (TrapFrame *)(task_stack_base - sizeof(TrapFrame));
  tf->elr_el1   = (uint64_t)function;
  tf->spsr_el1  = 0;

  auto &td = task_descriptors[tid] = {.tid        = tid,
                                      .parent_tid = -1,
                                      .priority   = priority,
                                      .state      = TaskStatus::READY,
                                      .sp_el0     = (uint64_t)tf};

  Kernel::scheduler.schedule(td);
  return td.tid;
}

Syscall activate(int tid) {
  TaskDescriptor &td = Kernel::task_descriptors[tid];
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
  TaskDescriptor &td = task_descriptors[tid];
  TrapFrame *tf      = (TrapFrame *)td.sp_el0;

  switch (request) {
  case Syscall::CREATE: {
    int new_tid            = _create(tf->x[0], (void (*)())tf->x[1]);
    tf->x[0]               = new_tid;
    TaskDescriptor &new_td = task_descriptors[new_tid];
    new_td.parent_tid      = tid;
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
    task_allocator.free(tid);
    break;
  }
  case Syscall::SEND: {
    int to_tid = tf->x[0];

    if (to_tid < 0 || to_tid >= MAX_TASKS) {
      tf->x[0] = -1; // invalid tid
      scheduler.schedule(td);
      break;
    }

    auto &to_td = task_descriptors[to_tid];

    if (to_td.state == TaskStatus::TERMINATED) {
      tf->x[0] = -1; // task doesn't exist
      scheduler.schedule(td);
      break;
    }

    if (to_tid == tid) {
      tf->x[0] = -2; // can't send to self
      scheduler.schedule(td);
      break;
    }

    if (to_td.state == TaskStatus::W4_SEND) {
      auto to_tf = (TrapFrame *)to_td.sp_el0;

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
      td.state    = TaskStatus::W4_REPLY;
      to_td.state = TaskStatus::READY;
      scheduler.schedule(to_td);
    } else {
      td.state = TaskStatus::W4_RECEIVE;
      to_td.sender_queue.push(tid);
    }

    break;
  }
  case Syscall::RECEIVE: {
    if (!td.sender_queue.is_empty()) {
      int from_tid = td.sender_queue.peek().value();
      td.sender_queue.pop();

      auto &to_td  = task_descriptors[from_tid];
      auto from_tf = (TrapFrame *)to_td.sp_el0;

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
      to_td.state = TaskStatus::W4_REPLY;
      td.state    = TaskStatus::READY;
      scheduler.schedule(td);
    } else {
      td.state = TaskStatus::W4_SEND;
    }
    break;
  }
  case Syscall::REPLY: {
    int to_tid = tf->x[0];

    if (to_tid < 0 || to_tid >= MAX_TASKS) {
      tf->x[0] = -1; // invalid tid
      scheduler.schedule(td);
      break;
    }

    auto &to_td = task_descriptors[to_tid];
    if (to_td.state != TaskStatus::W4_REPLY) {
      tf->x[0] = -2; // task not waiting for reply
      scheduler.schedule(td);
      break;
    }

    auto to_tf        = (TrapFrame *)to_td.sp_el0;
    const char *reply = (const char *)tf->x[1];
    int reply_len     = tf->x[2];

    char *rcv_reply = (char *)to_tf->x[3];
    int rcv_len     = to_tf->x[4];
    int len = to_tf->x[0] = tf->x[0] = std::min(reply_len, rcv_len);
    __builtin_memcpy(rcv_reply, reply, len);

    _assert(to_td.state == TaskStatus::W4_REPLY,
            "TASK NOT WAITING FOR REPLY\r\n");

    to_td.state = TaskStatus::READY;
    scheduler.schedule(to_td);
    scheduler.schedule(td);
    break;
  }
  }
  return;
}
