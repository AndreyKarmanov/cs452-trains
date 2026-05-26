#include <cstddef>
#include <cstring>
#include <ctype.h>

#include "uart.h"
#include "syscall.h"

#define BUFFER_SIZE 32

void fire_command(const char* buf, size_t blen) {
    if (blen == 0) return;
    if (strncmp(buf, "q", 1) == 0) {
        Exit();
    } else if (strncmp(buf, "p", 1) == 0) {
        int parent_tid = MyParentTid();
        uart_printf(CONSOLE, "My parent tid is %d\n\r", parent_tid);
    } else if (strncmp(buf, "m", 1) == 0) {
        int my_tid = MyTid();
        uart_printf(CONSOLE, "My tid is %d\n\r", my_tid);
    } else if (strncmp(buf, "y", 1) == 0) {
        Yield();
        uart_puts(CONSOLE, "Yielded\n\r");
    } else if (strncmp(buf, "c", 1) == 0) {
        int tid = Create(0, shell);
        uart_printf(CONSOLE, "Created task with tid %d\n\r", tid);
    } else {
        uart_puts(CONSOLE, "Unknown command. Available: q (quit), p (parent tid), m (my tid), y (yield), c (create)\n\r");
    }
}

void shell() {
    char buf[BUFFER_SIZE];
    size_t buf_n = 0;
    uart_puts(CONSOLE, "COMMANDS: q (quit) p (parent tid) m (my tid) y (yield) c (create)\n\r> ");
    while (1) {
        char c = uart_getc(CONSOLE);
        if (isprint(c) && buf_n < BUFFER_SIZE - 1) {
            buf[buf_n++] = c;
            uart_putc(CONSOLE, c);
        } else if ((c == 0x08 || c == 0x7f) && buf_n > 0) { // backspace
            uart_puts(CONSOLE, "\b \b"); // move back, print space, move back again
            --buf_n;
        } else if (c == '\r') { // enter
            buf[buf_n] = '\0';
            fire_command(buf, buf_n);
            uart_puts(CONSOLE, "> ");
            buf_n = 0;
            break;
        }
    }
}