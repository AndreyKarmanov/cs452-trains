#pragma once

#include <stdint.h>

#define TASK_STACK_SIZE 4096
#define TASK_DESCRIPTORS 4

enum class TaskState { READY, RUNNING, TERMINATED };

struct TaskDescriptor {
  int tid;
  int parent_tid;
  int priority;
  TaskState state;
  uint64_t sp_el0;   // this is the stack pointer
  uint64_t elr_el1;  // this is where to jump after kernel code is done
  uint64_t spsr_el1; // Program stat, including condition codes, interrupt

  uint8_t *stack_base;
};
