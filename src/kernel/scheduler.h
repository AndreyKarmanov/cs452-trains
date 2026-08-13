#pragma once

#include "buffer.h"
#include "debug.h"
#include "kernel/constants.h"
#include "task_descriptor.h"
#include <cstddef>
#include <expected>
#include <optional>

template <size_t MAX_TASKS, size_t MAX_PRIORITY> class Scheduler {
  Buffer<TaskId, MAX_TASKS> schedules[MAX_PRIORITY]{};

public:
  void schedule(TaskDescriptor &td) {
    panic_if(td.priority >= MAX_PRIORITY, "invalid scheduler priority");
    const bool pushed = schedules[td.priority].push(td.tid);
    panic_if(!pushed, "scheduler queue full");
  }

  void schedule(TaskId tid, TaskPriority priority) {
    panic_if(priority >= MAX_PRIORITY, "invalid scheduler priority");
    const bool pushed = schedules[priority].push(tid);
    panic_if(!pushed, "scheduler queue full");
  }

  Result<TaskId> get_task() {
    for (size_t i = 0; i < MAX_PRIORITY; i++) {
      auto res = schedules[i].pop();
      if (res.has_value()) {
        return res.value();
      }
    }
    return std::unexpected(KernelError::NotFound);
  }
};
