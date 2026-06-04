#pragma once

#include "kernel_state.h"
#include "syscall.h"

extern "C" Kernel::TrapFrame *_switch_to_user(uint64_t sp); // in boot.S
extern "C" void default_handler(int n);

int _create(int priority, void (*function)());
Syscall activate(int tid);

void handle(int tid, Syscall request);