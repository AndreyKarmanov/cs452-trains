#pragma once

#include "buffer.h"
#include "task_descriptor.h"

class TaskAllocator {
public:
  TaskAllocator(TaskDescriptor *task_descriptors);
  int get_new_task();
  int release_task(int tid);

  Buffer<int, TASK_DESCRIPTORS> free_tasks;

private:
  TaskDescriptor *task_descriptors;
};
