#pragma once

#include "buffer.h"
#include "kernel/constants.h"
#include <cstdint>

enum class TaskStatus {
  READY,
  RUNNING,
  W4_RECEIVE,
  W4_REPLY,
  W4_SEND,
  TERMINATED
};

struct alignas(16) TaskDescriptor {
  int td_idx;
  TaskId tid;
  TaskId parent_tid;
  TaskPriority priority;
  TaskStatus state;
  Buffer<int, 16> sender_queue{};
  uint64_t sp_el0;
};

struct alignas(16) TrapFrame {
  uint64_t x[31]; // x0 to x30
  uint64_t esr_el1;
  uint64_t elr_el1;
  uint64_t spsr_el1;
  uint64_t is_interrupt;
};
static_assert(sizeof(TrapFrame) == 288, "TrapFrame size must match boot.S");
