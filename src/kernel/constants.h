#pragma once
#include <cstddef>
#include <cstdint>

using TaskId       = int;
using TaskPriority = size_t;

inline constexpr size_t MAX_TASKS         = 32;
inline constexpr size_t PRIORITY_LEVELS   = 8;
inline constexpr uint64_t TASK_STACK_SIZE = 1048576;
