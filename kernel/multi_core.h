#pragma once

#include "task_descriptor.h"
#include <cstdint>

extern "C" char _start;

// Wakes a core from raspi_spintables by writing its entry address to the
// spin-table slot and sending SEV.
// x0: core id / MPIDR low bits
// x1: start address
extern "C" void wakeup_core(int core, uint64_t start_addr); // in boot.S

// Secondary-core handoff after the core is already running our boot code.
// x0: core # (1-3)
// x1: trapframe pointer (sp_el0)
extern "C" void core_entry(int core, uint64_t sp_el0); // in boot.S

// Helper: launches a task created by _create() on a secondary core.
// The task will never return to the scheduler and should loop internally.
//
// Example:
//   int tid = _create(priority, my_pinned_task_function);
//   launch_pinned_task(1, require_td(tid));
//
inline void launch_pinned_task(int core, const TaskDescriptor &td) {
  wakeup_core(core, reinterpret_cast<uint64_t>(&_start));
  core_entry(core, td.sp_el0);
}
