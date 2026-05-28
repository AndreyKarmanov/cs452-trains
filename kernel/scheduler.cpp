
#include "scheduler.h"
#include "debug.h"

void Scheduler::schedule(int tid, int priority) {
  // assert that priority is valid
  _assert(priority >= 0 && priority < MAX_PRIORITY, "invalid priority");

  // schedule the task
  schedules[priority].push(tid);
}

// returns the highest priorty task, if it exists and -1 if no tasks avilable.
int Scheduler::get_task() {
  for (int i = 0; i < MAX_PRIORITY; i++) {
    if (!schedules[i].is_empty()) {
      auto res = schedules[i].peek();
      if (!res.has_value())
        return -1;
      schedules[i].pop();
      return res.value();
    }
  }
  return -1;
}
