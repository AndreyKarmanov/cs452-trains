#pragma once

#include <cstdint>

enum class TaskStatus { READY, RUNNING, TERMINATED };

struct TaskDescriptor {
  int tid;
  int parent_tid;
  int priority;
  TaskStatus state;
  uint64_t sp_el0;
};
