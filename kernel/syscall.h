#pragma once

enum class Syscall {
  CREATE        = 0,
  MY_TID        = 1,
  MY_PARENT_TID = 2,
  YIELD         = 3,
  EXIT          = 4,
  SEND          = 5,
  RECEIVE       = 6,
  REPLY         = 7
};

int create(int priority, void (*function)());
int my_tid();
int my_parent_tid();
void yield();
void exit();

int send(int tid, const char *msg, int msglen, char *reply, int rplen);
int receive(int *tid, char *msg, int msglen);
int reply(int tid, const char *reply, int rplen);