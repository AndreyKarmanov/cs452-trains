#include "syscall.h"

int create(int priority, void (*function)()) {
  // this Create will trap to the kernel
  // and the kernel will return the tid of the created task
  // explicitly store in these registers
  register int r0 asm("x0") = priority;
  register void (*r1)() asm("x1") = function;
  asm volatile("svc %1" : "=r"(r0) : "i"(Syscall::CREATE) : "memory");
  return r0;
}

int my_tid() {
  // tells the compliler that we want this value to be stored directly in x0
  // https://gcc.gnu.org/onlinedocs/gcc/Local-Register-Variables.html
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
  // asm volatile("ret");
  return r0;
}

void yield() { asm volatile("svc %0" : : "i"(Syscall::YIELD) : "memory"); }

void exit() { asm volatile("svc %0" : : "i"(Syscall::EXIT) : "memory"); }

int send(int tid, const char* msg, int msg_len, char* reply, int reply_len) {
  register int r0 asm("x0") = tid;
  register const char* r1 asm("x1") = msg;
  register int r2 asm("x2") = msg_len;
  register char* r3 asm("x3") = reply;
  register int r4 asm("x4") = reply_len;

  asm volatile("svc %6"
               : "=r"(r0)
               : "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "i"(Syscall::SEND)
               : "memory");
  return r0;
};

int receive(int *tid, char* msg, int msg_len) {
  register int* r0_in asm("x0") = tid;
  register char* r1 asm("x1") = msg;
  register int r2 asm("x2") = msg_len;
  register int r0_out asm("x0");

  asm volatile("svc %4"
               : "=r"(r0_out)
               : "r"(r0_in), "r"(r1), "r"(r2), "i"(Syscall::RECEIVE)
               : "memory");
  return r0_out;
};

int reply(int tid, const char* reply, int reply_len) {
  register int r0 asm("x0") = tid;
  register const char* r1 asm("x1") = reply;
  register int r2 asm("x2") = reply_len;

  asm volatile("svc %4"
               : "=r"(r0)
               : "r"(r0), "r"(r1), "r"(r2), "i"(Syscall::REPLY)
               : "memory");
  return r0;
};