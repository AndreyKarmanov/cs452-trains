#include <cstdint>
#include <cstring>

#include "rpi.h"
#include "scheduler.h"
#include "shell.h"
#include "task_allocator.h"
#include "task_descriptor.h"
#include "uart.h"

extern "C" void setup_mmu(); // in mmu.S

namespace Kernel {
  struct KernelState {
    uint64_t sp;
    uint64_t x[12]; // x19 to x30 are callee-saved registers, so we need to save
                    // them in the kernel state
  } state;

  struct TrapFrame {
    uint64_t x[31]; // x0 to x30
    uint64_t sp;
    uint64_t elr_el1;
    uint64_t spsr_el1;
  };

  // This can live in the data section alongside other kernel data
  TaskDescriptor task_descriptors[TASK_DESCRIPTORS];
  TaskAllocator task_allocator(task_descriptors);
  int active_tid = task_allocator.get_new_task();
  Scheduler scheduler;

  // Make sure this lives in a separate, non-kernel section
  uint8_t task_stacks[TASK_DESCRIPTORS][TASK_STACK_SIZE]
      __attribute__((section(".task_stacks")));
} // namespace Kernel

extern "C" Kernel::TrapFrame *_switch_to_user(uint64_t sp); // in boot.S
extern "C" void default_handler() {
  uart_puts(CONSOLE, "DEFAULT VBAR HANDLER HIT\n\r");
}

int _create(int priority, void (*function)()) {
  // kernel side handler of the create systemcall
  // finds an empty task descriptor, fills with appropriate values
  // and returns the tid of the created task
  using namespace Kernel;
  auto &td = task_descriptors[0] = {
      .tid        = 0,
      .parent_tid = -1, // no parent
      .priority   = priority,
      .state      = TaskStatus::READY,
      .sp_el0     = (uint64_t)&task_stacks[0][TASK_STACK_SIZE],
      .elr_el1    = (uint64_t)function,
      .spsr_el1   = 0,
      .stack_base = &task_stacks[0][TASK_STACK_SIZE],
  };

  __builtin_memset((void *)td.sp_el0, 0, TASK_STACK_SIZE);

  // set the sp_el0, elr_el1 and spsr_el1 regions to right values

  TrapFrame *tf = (TrapFrame *)(td.sp_el0 - sizeof(TrapFrame));
  tf->sp        = td.sp_el0;
  tf->elr_el1   = (uint64_t)function;
  tf->spsr_el1  = 0; // set to 0 for now, can set to different values for
                     // different tasks if needed

  td.sp_el0 = (uint64_t)tf; // SAVE it back to the task!

  return td.tid;
}

int _activate(int tid) {
  // this will trap to the kernel and the kernel will perform a context switch
  // to the task with the given tid when the task yields or makes a syscall, it
  // will trap back to the kernel and return a request code that the task is
  // making to the kernel (syscalls) save x19 to x30 in kernel state, sp

  TaskDescriptor &td = Kernel::task_descriptors[tid];
  td.state           = TaskStatus::RUNNING;

  Kernel::TrapFrame *tf = _switch_to_user(td.sp_el0);
  td.sp_el0             = (uint64_t)tf;

  return tf->x[0];
}

int _handle(int /*tid*/, int /*request*/) {
  // this will handle the given request code and perform the appropriate action
  // (e.g. for syscalls) ESR_EL1 will have exception code, holds n form svc N
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

  for (size_t i = 0; i < TASK_DESCRIPTORS; i++) {
    uart_printf(CONSOLE, "Task %u stack: 0x%x\n\r", i, &Kernel::task_stacks[i]);
  }
  _create(0, shell);

  // for now we will have only 1 task
  for (;;) {
    int request = _activate(0);
    uart_printf(CONSOLE, "Request code: %d\n\r", request);
    _handle(0, request);
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
