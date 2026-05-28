#include <cstdint>

#include "syscall.h"
#include "uart.h"

int create(int /*priority*/, void (* /*function*/)()) {
  // this Create will trap to the kernel
  // and the kernel will return the tid of the created task
  return 0;
}

int my_tid() { return 0; }

int my_parent_tid() { return 0; }

void yield() {
  uint64_t user_reg_30a = 0;
  uint64_t user_reg_30b = 0;

  asm volatile("mov %0, x19" : "=r"(user_reg_30a));
  uart_printf(CONSOLE, "Yield: user_reg_30a = %x, user_reg_30b = %x\n\r",
              user_reg_30a, user_reg_30b);

  asm volatile("mov x0, #50\n\t");
  asm volatile("svc #0");
  asm volatile("mov %0, x19" : "=r"(user_reg_30b));

  uart_printf(CONSOLE, "Yield: user_reg_30a = %x, user_reg_30b = %x\n\r",
              user_reg_30a, user_reg_30b);
}

void exit() {}