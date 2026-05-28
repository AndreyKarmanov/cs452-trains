#include "task_allocator.h"

TaskAllocator::TaskAllocator(TaskDescriptor *task_descriptors)
    : task_descriptors(task_descriptors) {
  for (int i = 0; i < TASK_DESCRIPTORS; i++) {
    free_tasks.push(i);
  }
}

int TaskAllocator::get_new_task() {
  if (free_tasks.is_empty()) { // unable to allocate new tasks
    return -1;
  }
  int tid = free_tasks.peek();
  free_tasks.pop();
  return tid;
}

int TaskAllocator::release_task(int tid) {
  if (tid < 0 || tid >= TASK_DESCRIPTORS) {
    return -1;
  }
  free_tasks.push(tid);
  return 0;
}