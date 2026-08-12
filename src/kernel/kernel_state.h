#pragma once

#include "event_controller.h"
#include "idle_manager.h"
#include "kernel/constants.h"
#include "scheduler.h"
#include "task_table.h"
#include <cstdint>

inline struct KernelRuntime {
  EventController event_controller;
  TaskTable task_table;
  Scheduler<MAX_TASKS, PRIORITY_LEVELS> scheduler;
  IdleManager idle_manager;
} kernel_runtime;

inline uint8_t task_stacks[MAX_TASKS][TASK_STACK_SIZE]
    __attribute__((section(".task_stacks")));