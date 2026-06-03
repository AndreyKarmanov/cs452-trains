#pragma once
#include <cstdint>

#include "allocator.h"
#include "scheduler.h"
#include "task_descriptor.h"

#define PRIORITY_LEVELS 4
#define MAX_TASKS 16
#define TASK_STACK_SIZE 4096

namespace Kernel {
  struct TrapFrame {
    uint64_t x[31]; // x0 to x30
    uint64_t esr_el1;
    uint64_t elr_el1;
    uint64_t spsr_el1;
  };

  // inline variable here says "this is a global"
  // even if you include multiple times, just use this one
  inline TaskDescriptor task_descriptors[MAX_TASKS];
  inline Allocator<MAX_TASKS> task_allocator;
  inline Scheduler<MAX_TASKS, PRIORITY_LEVELS> scheduler;

  // Make sure this lives in a separate, non-kernel section
  inline uint8_t task_stacks[MAX_TASKS][TASK_STACK_SIZE]
      __attribute__((section(".task_stacks")));
} // namespace Kernel