#pragma once

enum class Syscall {
  CREATE        = 0,
  MY_TID        = 1,
  MY_PARENT_TID = 2,
  YIELD         = 3,
  EXIT          = 4,
};

int create(int priority, void (*function)());
int my_tid();
int my_parent_tid();
void yield();
void exit();
