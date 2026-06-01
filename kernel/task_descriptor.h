#pragma once

#include "buffer.h"
#include <cstdint>

enum class TaskStatus { READY, RUNNING, W4_RECEIVE, W4_REPLY, W4_SEND, TERMINATED };


// TODO: perhaps make buffer a linkedlist?, update the sender queue to be templated? idk.
struct TaskDescriptor {
  int tid;
  int parent_tid;
  int priority;
  TaskStatus state;
  Buffer<int, 16> sender_queue;
  uint64_t sp_el0;
};
