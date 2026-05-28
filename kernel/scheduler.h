#pragma once

#include <optional>

#include "buffer.h"
#include "debug.h"

template <size_t MAX_TASKS, size_t MAX_PRIORITY> class Scheduler {
public:
  void schedule(int tid, int priority) {
    // assert that priority is valid
    _assert(priority >= 0 && priority < MAX_PRIORITY, "invalid priority");

    // schedule the task
    schedules[priority].push(tid);
  };

  std::optional<int> get_task() {
    for (int i = 0; i < MAX_PRIORITY; i++) {
      if (!schedules[i].is_empty()) {
        auto res = schedules[i].peek();
        if (!res.has_value())
          return std::nullopt;
        schedules[i].pop();
        return res.value();
      }
    }
    return std::nullopt;
  };

private:
  Buffer<int, MAX_TASKS> schedules[MAX_PRIORITY]{};
};
