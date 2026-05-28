#pragma once

#include "buffer.h"
#include "task_descriptor.h"
#include <optional>

template <size_t MAX_TASKS> class TaskStackAllocator {
  Buffer<int, MAX_TASKS> free_tasks;

public:
  // constexpr so we don't have to initalize at runtime
  constexpr TaskStackAllocator() {
    for (size_t i = 0; i < MAX_TASKS; i++) {
      free_tasks.push(i);
    }
  }

  constexpr std::optional<int> get_new_task() {
    auto tid = free_tasks.peek();
    if (!tid.has_value()) {
      return std::nullopt;
    }
    free_tasks.pop();
    return tid.value();
  }

  constexpr bool free_task(int tid) {
    if (tid < 0 || tid >= MAX_TASKS) {
      return false; // invalid tid
    }
    return free_tasks.push(tid);
  }
};
