#include <cstdint>
#include <cstring>
#include <optional>

#include "rpi.h"
#include "scheduler.h"
#include "shell.h"
#include "task_allocator.h"
#include "task_descriptor.h"
#include "uart.h"

extern "C" void setup_mmu(); // in mmu.S

#define PRIORITY_LEVELS 3
#define MAX_TASKS 4
#define TASK_STACK_SIZE 4096

namespace Kernel {
  struct TrapFrame {
    uint64_t x[31]; // x0 to x30
    uint64_t sp;
    uint64_t elr_el1;
    uint64_t spsr_el1;
  };

  TaskDescriptor task_descriptors[MAX_TASKS];
  TaskStackAllocator<MAX_TASKS> task_allocator;
  Scheduler<MAX_TASKS, PRIORITY_LEVELS> scheduler;

  int active_tid = -1;

  // Make sure this lives in a separate, non-kernel section
  uint8_t task_stacks[MAX_TASKS][TASK_STACK_SIZE]
      __attribute__((section(".task_stacks")));
} // namespace Kernel

extern "C" Kernel::TrapFrame *_switch_to_user(uint64_t sp); // in boot.S
extern "C" void default_handler() {
  uart_puts(CONSOLE, "DEFAULT VBAR HANDLER HIT\n\r");
}

// Allocates a new task, initalizes descriptor and stack
int _create(int priority, void (*function)()) {
  using namespace Kernel;

  if (0 > priority || priority >= PRIORITY_LEVELS) {
    return -1; // invalid priority
  }

  auto tid_opt = task_allocator.get_new_task();
  if (tid_opt == std::nullopt) {
    return -2; // no free task descriptors
  }
  auto tid = tid_opt.value();

  auto &td = task_descriptors[tid] = {
      .tid        = tid,
      .parent_tid = -1,
      .priority   = priority,
      .state      = TaskStatus::READY,
      .sp_el0     = (uint64_t)&task_stacks[tid][TASK_STACK_SIZE],
  };

  // clear stack memory (not required but helpful)
  __builtin_memset((void *)td.sp_el0, 0, TASK_STACK_SIZE);

  // build & push inital trapframe
  TrapFrame *tf = (TrapFrame *)(td.sp_el0 - sizeof(TrapFrame));
  tf->sp        = td.sp_el0;
  tf->elr_el1   = (uint64_t)function;
  tf->spsr_el1  = 0;

  // set stack pointer to top of trapframe
  td.sp_el0 = (uint64_t)tf;

  return td.tid;
}

int _activate(int tid) {
  TaskDescriptor &td = Kernel::task_descriptors[tid];
  td.state           = TaskStatus::RUNNING;

  // switch to user mode
  // this will return when task makes a syscall
  Kernel::TrapFrame *tf = _switch_to_user(td.sp_el0);
  td.sp_el0             = (uint64_t)tf;

  // this is for us to decide how to encode the syscall
  // for now we are just reading x0
  return tf->x[0];
}

int _handle(int tid, int request) {
  // this will handle the given request code and perform the appropriate action
  // (e.g. for syscalls) ESR_EL1 will have exception code, holds n form svc N
  uart_printf(CONSOLE, "%d requested %d\n\r", tid, request);
  return 0;
}

extern "C" int kmain() {
#if defined(MMU)
  setup_mmu();
#endif
  gpio_init();
  uart_config_and_enable(CONSOLE);

  uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__
                     " / Andrey Karmanov / Anthony Ho\n\r");

  for (size_t i = 0; i < MAX_TASKS; i++) {
    uart_printf(CONSOLE, "Task %u stack: 0x%x\n\r", i, &Kernel::task_stacks[i]);
  }

  Kernel::scheduler.schedule(_create(0, shell), 0);

  for (;;) {
    auto tid = Kernel::scheduler.get_task();
    if (!tid.has_value()) {
      continue; // no ready tasks, spin
    }
    auto active_tid = tid.value();

    int request = _activate(active_tid);
    _handle(active_tid, request);
  }

  return 0;
}

#if !defined(MMU)
#include <cstddef>

// define our own memset to avoid SIMD instructions emitted from the compiler
void *memset(void *s, int c, size_t n) {
  for (char *it = (char *)s; n > 0; --n)
    *it++ = c;
  return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void *memcpy(void *dest, const void *src, size_t n) {
  char *sit   = (char *)src;
  char *cdest = (char *)dest;
  for (size_t i = 0; i < n; ++i)
    *cdest++ = *sit++;
  return dest;
}
#endif
