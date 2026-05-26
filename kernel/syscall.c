#include "syscall.h"

int Create(int priority, void (*function)()) {
    // this Create will trap to the kernel
    // and the kernel will return the tid of the created task
    return 0;
}

int MyTid() {
    return 0;
}

int MyParentTid() {
    return 0;
}

void Yield() {

}

void Exit() {

}