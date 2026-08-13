#include "task_table.h"
#include "kernel/constants.h"
#include "kernel/kernel_state.h"

extern "C" void task_entry_wrapper(void (*function)()) {
  function();
  exit();
}

Result<TaskId> TaskTable::create_task(TaskPriority priority, void (*function)(),
                                      TaskId parent_tid) {
  if (priority >= PRIORITY_LEVELS) {
    return std::unexpected(KernelError::InvalidPriority);
  }

  auto descriptor_index_opt = allocator.allocate();
  if (!descriptor_index_opt.has_value()) {
    return std::unexpected(KernelError::NoFreeTasks);
  }
  auto td_idx = descriptor_index_opt.value();

  uint64_t task_stack_base =
      reinterpret_cast<uint64_t>(&task_stacks[td_idx][TASK_STACK_SIZE]);
  uint64_t task_stack_end = task_stack_base - TASK_STACK_SIZE;
  std::fill_n(reinterpret_cast<uint8_t *>(task_stack_end), TASK_STACK_SIZE, 0);

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

  if (!tid_to_descriptor.set(tid, td_idx)) {
    allocator.free(td_idx);
    return std::unexpected(KernelError::NoFreeTasks);
  }

  return tid;
}

Result<TaskDescriptor *> TaskTable::lookup_td(TaskId tid) {
  auto td_idx_opt = tid_to_descriptor.get(tid);
  if (!td_idx_opt.has_value()) {
    return std::unexpected(KernelError::InvalidTid);
  }
  auto td_idx = td_idx_opt.value();
  return &descriptors[td_idx];
}

void TaskTable::delete_task(TaskId tid) {
  auto td_idx_opt = tid_to_descriptor.get(tid);
  if (!td_idx_opt.has_value()) {
    return;
  }
  auto td_idx = td_idx_opt.value();
  tid_to_descriptor.remove(tid);
  allocator.free(td_idx);
  return;
}