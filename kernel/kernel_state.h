#pragma once

#include "allocator.h"
#include "buffer.h"
#include "debug.h"
#include "idle_manager.h"
#include "map.h"
#include "scheduler.h"
#include "syscall.h"
#include "task_descriptor.h"
#include <cstdint>
#include <optional>

#define PRIORITY_LEVELS 8
#define MAX_TASKS 16
#define TASK_STACK_SIZE 4096

namespace Kernel {
  struct alignas(16) TrapFrame {
    uint64_t x[31]; // x0 to x30
    uint64_t esr_el1;
    uint64_t elr_el1;
    uint64_t spsr_el1;
    uint64_t is_interrupt;
  };

  static_assert(sizeof(TrapFrame) == 288, "TrapFrame size must match boot.S");

  // inline variable here says "this is a global"
  // even if you include multiple times, just use this one
  inline TaskDescriptor task_descriptors[MAX_TASKS];
  inline Allocator<MAX_TASKS> task_allocator;
  inline Map<int, int, MAX_TASKS> tid_to_descriptor;
  inline int next_tid = 0;

  inline Scheduler<MAX_TASKS, PRIORITY_LEVELS> scheduler;

  inline Map<Event, Buffer<int, MAX_TASKS>, TOTAL_EVENT_TYPES> event_buffers;
  inline uint64_t initalized_events{0};

  inline IdleManager idle_manager;
  inline Map<Syscall, int, 32> syscall_cycle_totals;
  inline Map<Syscall, int, 32> syscall_cycle_counts;

  // Make sure this lives in a separate, non-kernel section
  inline uint8_t task_stacks[MAX_TASKS][TASK_STACK_SIZE]
      __attribute__((section(".task_stacks")));

  inline std::optional<TaskDescriptor *> lookup_td(int tid) {
    auto descriptor_index_opt = tid_to_descriptor.get(tid);
    if (!descriptor_index_opt.has_value()) {
      _assert(false, "invalid tid");
      return std::nullopt;
    }

    return &task_descriptors[descriptor_index_opt.value()];
  }
} // namespace Kernel