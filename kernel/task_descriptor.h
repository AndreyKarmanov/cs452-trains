#pragma once

#include <cstdint>

#include "buffer.h"

enum class TaskStatus {
  READY,
  RUNNING,
  W4_RECEIVE,
  W4_REPLY,
  W4_SEND,
  TERMINATED
};

// TODO: perhaps make buffer a linkedlist?, update the sender queue to be
// templated? idk.
struct alignas(16) TaskDescriptor {
  int td_idx;
  int tid;
  int parent_tid;
  int priority;
  TaskStatus state;
  Buffer<int, 16> sender_queue{};
  uint64_t sp_el0;
};
