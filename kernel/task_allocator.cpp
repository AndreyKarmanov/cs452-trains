#include "task_allocator.h"

TaskAllocator::TaskAllocator(TaskDescriptor *task_descriptors)
    : task_descriptors(task_descriptors) {
  for (int i = 0; i < TASK_DESCRIPTORS; i++) {
    free_tasks.push(i);
  }
}

int TaskAllocator::get_new_task() {
  auto tid = free_tasks.peek();
  if (tid == std::nullopt) {
    return -1;
  }
  free_tasks.pop();
  return tid.value();
}

int TaskAllocator::release_task(int tid) {
  if (tid < 0 || tid >= TASK_DESCRIPTORS) {
    return -1;
  }
  free_tasks.push(tid);
  return 0;
}