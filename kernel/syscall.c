#include "syscall.h"
#include "uart.h"
#include <stdint.h>

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
    uint64_t user_reg_30a = 0;
    uint64_t user_reg_30b = 0;

    asm volatile("mov %0, x19" : "=r"(user_reg_30a));
    uart_printf(CONSOLE, "Yield: user_reg_30a = %x, user_reg_30b = %x\n\r", user_reg_30a, user_reg_30b);


    asm volatile("mov x0, #0\n\t");
    asm volatile("svc #0");
    asm volatile("mov %0, x19" : "=r"(user_reg_30b));

    uart_printf(CONSOLE, "Yield: user_reg_30a = %x, user_reg_30b = %x\n\r", user_reg_30a, user_reg_30b);
}

void Exit() {

}