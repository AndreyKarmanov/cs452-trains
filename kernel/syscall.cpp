#include "syscall.h"

int create(int /*priority*/, void (* /*function*/)()) {
  // this Create will trap to the kernel
  // and the kernel will return the tid of the created task
  asm volatile("svc %0" : : "i"(Syscall::CREATE));
  return 0;
}

int my_tid() {
  // tells the compliler that we want this value to be stored directly in x0
  register int r0 asm("x0");

  /*
  volatile: tell g++ don't touch this code
  "=": this will be written to
  "r": this should be a register
  "i": this is an immediate value (constant)
  asm volatile (
  "code"
  : outputs
  : inputs
  : clobbered values (e.g. registers or memory)
  )
  */
  asm volatile("svc %0" : "=r"(r0) : "i"(Syscall::MY_TID));
  return r0;
}

int my_parent_tid() {
  asm volatile("svc %0" : : "i"(Syscall::MY_PARENT_TID));
  register int r0 asm("x0");
  return r0;
}

void yield() { asm volatile("svc %0" : : "i"(Syscall::YIELD)); }

void exit() { asm volatile("svc %0" : : "i"(Syscall::EXIT)); }