#pragma once

#include "buffer.h"
#include "task_descriptor.h"
#include <cstddef>
#include <optional>

template <size_t MAX_TASKS, size_t MAX_PRIORITY> class Scheduler {
  Buffer<int, MAX_TASKS> schedules[MAX_PRIORITY]{};
public:
  void schedule(TaskDescriptor &td) { schedules[td.priority].push(td.tid); };

  std::optional<int> get_task() {
    for (size_t i = 0; i < MAX_PRIORITY; i++) {
      auto res = schedules[i].pop();
      if (res.has_value()) {
        return res.value();
      }
    }
    return std::nullopt;
  };

};
