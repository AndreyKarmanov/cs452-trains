#include "idle_manager.h"
#include "syscall.h"
#include "time.h"

void idle_task() {
  for (;;) {
    park();
  }
}

IdleManager::IdleManager() : last_busy_us_(time_get()) {}

void IdleManager::go_idle() {
  // Note: should reason if we should double check that kernel scheduler is
  // empty here. That is, we don't go idle if there are tasks. Personally I
  // think it's excessive.

  // account for busy time
  const uint32_t start  = time_get();
  busy_us_             += static_cast<uint64_t>(start - last_busy_us_);

  // go idle and track idle time
  asm volatile("wfi" ::: "memory");
  const uint32_t end  = time_get();
  idle_us_           += static_cast<uint64_t>(end - start);

  // kernel is busy after wfi
  last_busy_us_ = time_get();
}

int IdleManager::get_idle_time_percentage() {
  const uint64_t total = idle_us_ + busy_us_;
  if (total == 0) {
    return 100;
  }
  return static_cast<int>((idle_us_ * 100) / total);
}
