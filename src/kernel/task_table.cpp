#include "task_table.h"
#include "kernel/constants.h"
#include "kernel/kernel_state.h"

extern "C" void task_entry_wrapper(void (*function)()) {
  function();
  exit();
}

TaskId TaskTable::create_task(TaskPriority priority, void (*function)(),
                              TaskId parent_tid) {
  if (priority >= PRIORITY_LEVELS) {
    panic("Invalid priority");
  }

  auto descriptor_index_opt = allocator.allocate();
  if (descriptor_index_opt == std::nullopt) {
    panic("No free task descriptors");
  }
  auto td_idx = descriptor_index_opt.value();

  // define task stack (grows downwards)
  uint64_t task_stack_base =
      reinterpret_cast<uint64_t>(&task_stacks[td_idx][TASK_STACK_SIZE]);
  uint64_t task_stack_end = task_stack_base - TASK_STACK_SIZE;

  // clear stack memory (not required but helpful)
  std::fill_n(reinterpret_cast<uint8_t *>(task_stack_end), TASK_STACK_SIZE, 0);

  // build & push inital trapframe
  TrapFrame *tf =
      reinterpret_cast<TrapFrame *>(task_stack_base - sizeof(TrapFrame));
  tf->elr_el1      = reinterpret_cast<uint64_t>(task_entry_wrapper);
  tf->x[0]         = reinterpret_cast<uint64_t>(function);
  tf->spsr_el1     = 0;
  tf->is_interrupt = 0;

  auto tid            = next_tid++;
  descriptors[td_idx] = {
      .td_idx     = td_idx,
      .tid        = tid,
      .parent_tid = parent_tid,
      .priority   = priority,
      .state      = TaskStatus::READY,
      .sp_el0     = reinterpret_cast<uint64_t>(tf),
  };
  tid_to_descriptor.set(tid, td_idx);
  return tid;
}

TaskDescriptor *TaskTable::lookup_td(TaskId tid) {
  auto td_idx_opt = tid_to_descriptor.get(tid);
  if (!td_idx_opt.has_value()) {
    return nullptr;
  }
  auto td_idx = td_idx_opt.value();
  return &descriptors[td_idx];
}

void TaskTable::delete_task(TaskId tid) {
  auto td_idx_opt = tid_to_descriptor.get(tid);
  if (!td_idx_opt.has_value()) {
    panic("Invalid tid");
  }
  auto td_idx = td_idx_opt.value();
  tid_to_descriptor.remove(tid);
  allocator.free(td_idx);
}