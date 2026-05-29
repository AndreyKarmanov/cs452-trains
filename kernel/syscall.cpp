#include "syscall.h"

int create(int /*priority*/, void (* /*function*/)()) {
  // this Create will trap to the kernel
  // and the kernel will return the tid of the created task
  register int r0 asm("x0");
  asm volatile("svc %1" : "=r"(r0) : "i"(Syscall::CREATE) : "memory");
  return r0;
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

  the below line is saying:
  execute svc SYSCALL::MY_TID
  and there will be a new value in r0
  and btw, all RAM might be changed
  (i.e. maybe uart came in while this was executed)
  */
  asm volatile("svc %1" : "=r"(r0) : "i"(Syscall::MY_TID) : "memory");
  return r0;
}

int my_parent_tid() {
  register int r0 asm("x0");
  asm volatile("svc %1" : "=r"(r0) : "i"(Syscall::MY_PARENT_TID) : "memory");
  return r0;
}

void yield() { asm volatile("svc %0" : : "i"(Syscall::YIELD) : "memory"); }

void exit() { asm volatile("svc %0" : : "i"(Syscall::EXIT) : "memory"); }