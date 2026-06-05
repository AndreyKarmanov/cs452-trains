#pragma once
#include <cstdint>

#include "allocator.h"
#include "map.h"
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
  inline Map<int, int, MAX_TASKS> tid_to_descriptor;
  inline uint64_t next_tid = 0;

  inline Scheduler<MAX_TASKS, PRIORITY_LEVELS> scheduler;

  // Make sure this lives in a separate, non-kernel section
  inline uint8_t task_stacks[MAX_TASKS][TASK_STACK_SIZE]
      __attribute__((section(".task_stacks")));

  inline TaskDescriptor *lookup_td(int tid) {
    using namespace Kernel;

    auto descriptor_index_opt = tid_to_descriptor.get(tid);
    if (!descriptor_index_opt.has_value()) {
      return nullptr;
    }

    return &task_descriptors[descriptor_index_opt.value()];
  }

  inline TaskDescriptor &require_td(int tid) {
    auto *td = lookup_td(tid);
    _assert(td != nullptr, "invalid tid");
    return *td;
  }
} // namespace Kernel