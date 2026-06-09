#pragma once

#include "task_descriptor.h"
#include <cstdint>

// Declares the low-level mailbox activation routine.
// x0: core # (1-3)
// x1: trapframe pointer (sp_el0) - 64-bit address to be split and sent via
// mailbox
//
// Implemented in boot.S; writes the trapframe address to the secondary core's
// mailbox and sends SEV. The secondary core wakes, reads the mailbox,
// transitions to EL1, enables the MMU, and enters EL0 to run the task.
extern "C" void core_entry(int core, uint64_t sp_el0); // in boot.S

// Helper: launches a task created by _create() on a secondary core.
// The task will never return to the scheduler and should loop internally.
//
// Example:
//   int tid = _create(priority, my_pinned_task_function);
//   launch_pinned_task(1, require_td(tid));
//
inline void launch_pinned_task(int core, const TaskDescriptor &td) {
  core_entry(core, td.sp_el0);
}
