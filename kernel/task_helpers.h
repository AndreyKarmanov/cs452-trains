#ifndef _task_helpers_h
#define _task_helpers_h

#include <stdint.h>

// everything that we need to save to account for context-switch
struct TrapFrame {
  // need to save x0 to x30 to allow for context switching
  uint64_t x[31];

  // also have to save the SP_EL0 and ELR_EL1 registers to allow for context
  // switching
  uint64_t sp_el0;    // this is the stack pointer
  uint64_t elr_el1;   // this is where to jump after kernel code is done
  uint64_t spsr_el1;  // Program stat, including condition codes, interrupt
                      // flags, execution state, etc.
};

enum class TaskState { READY, RUNNING, TERMINATED };

struct TaskDescriptor {
  int tid;
  int parent_tid;
  int priority;
  TaskState state;
  TrapFrame tf;
  uint8_t* stack_base;
};

#endif
