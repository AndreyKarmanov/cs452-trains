#ifndef _syscall_h_
#define _syscall_h_

int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();

#endif // _syscall_h_