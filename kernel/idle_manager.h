#pragma once

#include <cstdint>

// userspace helpers
void idle_task();

// kernelspace struct for managing idle task
class IdleManager {
public:
  IdleManager();
  void go_idle();
  int get_idle_time_percentage();

private:
  uint64_t last_busy_us_;
  uint64_t idle_us_{0};
  uint64_t busy_us_{0};
};
