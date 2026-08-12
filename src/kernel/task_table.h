#pragma once

#include "allocator.h"
#include "kernel/constants.h"
#include "map.h"
#include "task_descriptor.h"

struct TaskTable {
  TaskId next_tid{0};
  Allocator<MAX_TASKS> allocator{};
  std::array<TaskDescriptor, MAX_TASKS> descriptors{};
  Map<TaskId, size_t, MAX_TASKS> tid_to_descriptor{};

  TaskId create_task(TaskPriority priority, void (*function)(),
                     TaskId parent_tid);
  void delete_task(TaskId tid);
  TaskDescriptor *lookup_td(TaskId tid);
};
