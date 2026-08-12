#pragma once

#include "kernel_state.h"
#include "syscall.h"

extern "C" TrapFrame *_switch_to_user(uint64_t sp); // in boot.S
extern "C" void default_handler(int n);

Syscall activate_task(TaskDescriptor &td);

void handle(int tid, Syscall request);