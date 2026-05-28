#pragma once
#include "buffer.h"

class Scheduler {
public:
  void schedule(int tid, int priority);
  int get_task();
  static const int MAX_PRIORITY = 3; // total prio levels

private:
  Buffer<int, 64> schedules[MAX_PRIORITY]{};
};
