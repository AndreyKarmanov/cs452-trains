#pragma once

#include "buffer.h"
#include "kernel/constants.h"
#include "task_descriptor.h"
#include <cstddef>
#include <optional>

template <size_t MAX_TASKS, size_t MAX_PRIORITY> class Scheduler {
  Buffer<TaskId, MAX_TASKS> schedules[MAX_PRIORITY]{};

public:
  void schedule(TaskDescriptor &td) { schedules[td.priority].push(td.tid); };
  void schedule(TaskId tid, TaskPriority priority) {
    if (priority >= MAX_PRIORITY) {
      panic("invalid priority");
    }
    schedules[priority].push(tid);
  };

  std::optional<TaskId> get_task() {
    for (size_t i = 0; i < MAX_PRIORITY; i++) {
      auto res = schedules[i].pop();
      if (res.has_value()) {
        return res.value();
      }
    }
    return std::nullopt;
  };
};
