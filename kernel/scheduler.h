#pragma once

#include <optional>

#include "buffer.h"
#include "debug.h"
#include "task_descriptor.h"

template <size_t MAX_TASKS, size_t MAX_PRIORITY> class Scheduler {
public:
  void schedule(TaskDescriptor &td) {
    // assert that priority is valid
    _assert(td.priority >= 0 && td.priority < MAX_PRIORITY, "invalid priority");

    // schedule the task
    schedules[td.priority].push(td.tid);
  };

  std::optional<int> get_task() {
    for (int i = MAX_PRIORITY - 1; i >= 0; i--) {
      if (!schedules[i].is_empty()) {
        auto res = schedules[i].peek();
        schedules[i].pop();
        return res.value();
      }
    }
    return std::nullopt;
  };

private:
  Buffer<int, MAX_TASKS> schedules[MAX_PRIORITY]{};
};
