#include "first_user_task.h"
#include "shell.h"
#include "syscall.h"
#include "test.h"

void FirstUserTask::run() { create(2, shell); }

void first_user_task() {
  FirstUserTask fut{};
  fut.run();
}