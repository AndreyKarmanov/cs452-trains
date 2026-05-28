#include <cstdint>
#include <cstring>

#include "rpi.h"
#include "scheduler.h"
#include "shell.h"
#include "task_allocator.h"
#include "task_helpers.h"
#include "uart.h"

extern "C" void setup_mmu();                    // in mmu.S
extern "C" void _restore_user_stack_and_eret(); // in boot.S

struct KernelState {
  uint64_t sp;
  uint64_t x[12]; // x19 to x30 are callee-saved registers, so we need to save
                  // them in the kernel state
} kernel_state;

// This can live in the data section alongside other kernel data
TaskDescriptor task_descriptors[TASK_DESCRIPTORS];
TaskAllocator task_allocator(task_descriptors);
int active_tid = task_allocator.get_new_task();
Scheduler scheduler;

// Make sure this lives in a separate, non-kernel section
uint8_t task_stacks[TASK_DESCRIPTORS][TASK_STACK_SIZE]
    __attribute__((section(".task_stacks")));

extern "C" void default_handler() {
  uart_puts(CONSOLE, "DEFAULT VBAR HANDLER HIT\n\r");
}

int _create(int priority, void (*function)()) {
  // kernel side handler of the create systemcall
  // finds an empty task descriptor, fills with appropriate values
  // and returns the tid of the created task
  // write via asm - positional argument %0
  TaskDescriptor &td                  = task_descriptors[0];
  task_stacks[0][TASK_STACK_SIZE - 1] = 0;
  td.tid                              = 0;
  td.parent_tid                       = -1; // no parent
  td.priority                         = priority;
  td.state                            = TaskState::READY;

  td.stack_base = &task_stacks[0][TASK_STACK_SIZE];
  td.sp_el0     = (uint64_t)td.stack_base;
  td.elr_el1    = (uint64_t)function;
  td.spsr_el1   = 0;
  __builtin_memset((void *)td.sp_el0, 0, TASK_STACK_SIZE);
  td.sp_el0 -= 256; // shift by 256 bytes down, for our fake trap frame

  return 0;
}

extern "C" int lower_el_64_sync_handler() {
  TaskDescriptor &td = task_descriptors[active_tid];
  asm volatile("mrs %0, sp_el0" : "=r"(td.sp_el0));
  asm volatile("mrs %0, elr_el1" : "=r"(td.elr_el1));
  asm volatile("mrs %0, spsr_el1" : "=r"(td.spsr_el1));

  // restore kernal state
  asm volatile(
      "mov x19, %0\n\t"
      "mov x20, %1\n\t"
      "mov x21, %2\n\t"
      "mov x22, %3\n\t"
      "mov x23, %4\n\t"
      "mov x24, %5\n\t"
      "mov x25, %6\n\t"
      "mov x26, %7\n\t"
      "mov x27, %8\n\t"
      "mov x28, %9\n\t"
      "mov x29, %10\n\t"
      "mov x30, %11\n\t"
      "mov sp, %12\n\t" ::"r"(kernel_state.x[0]),
      "r"(kernel_state.x[1]), "r"(kernel_state.x[2]), "r"(kernel_state.x[3]),
      "r"(kernel_state.x[4]), "r"(kernel_state.x[5]), "r"(kernel_state.x[6]),
      "r"(kernel_state.x[7]), "r"(kernel_state.x[8]), "r"(kernel_state.x[9]),
      "r"(kernel_state.x[10]), "r"(kernel_state.x[11]), "r"(kernel_state.sp));

  asm volatile("ldr x0, [sp, #0]\n\t"
               "ret");

  __builtin_unreachable();
}

int _activate(int tid) {
  // this will trap to the kernel and the kernel will perform a context switch
  // to the task with the given tid when the task yields or makes a syscall, it
  // will trap back to the kernel and return a request code that the task is
  // making to the kernel (syscalls) save x19 to x30 in kernel state, sp
  asm volatile("mov %0, x19\n\t"
               "mov %1, x20\n\t"
               "mov %2, x21\n\t"
               "mov %3, x22\n\t"
               "mov %4, x23\n\t"
               "mov %5, x24\n\t"
               "mov %6, x25\n\t"
               "mov %7, x26\n\t"
               "mov %8, x27\n\t"
               "mov %9, x28\n\t"
               "mov %10, x29\n\t"
               "mov %11, x30\n\t"
               "mov %12, sp\n\t"
               : "=r"(kernel_state.x[0]), "=r"(kernel_state.x[1]),
                 "=r"(kernel_state.x[2]), "=r"(kernel_state.x[3]),
                 "=r"(kernel_state.x[4]), "=r"(kernel_state.x[5]),
                 "=r"(kernel_state.x[6]), "=r"(kernel_state.x[7]),
                 "=r"(kernel_state.x[8]), "=r"(kernel_state.x[9]),
                 "=r"(kernel_state.x[10]), "=r"(kernel_state.x[11]),
                 "=r"(kernel_state.sp));

  TaskDescriptor &td = task_descriptors[tid];
  td.state           = TaskState::RUNNING;

  asm volatile("msr sp_el0, %0" ::"r"(td.sp_el0));
  asm volatile("msr elr_el1, %0" ::"r"(td.elr_el1));
  asm volatile("msr spsr_el1, %0" ::"r"(td.spsr_el1));

  _restore_user_stack_and_eret();

  return 0;
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
    uart_printf(CONSOLE, "Task %u stack: 0x%x\n\r", i, &task_stacks[i]);
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
