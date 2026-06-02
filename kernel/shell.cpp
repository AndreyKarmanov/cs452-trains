#include <cstddef>
#include <cstring>
#include <ctype.h>

#include "shell.h"
#include "syscall.h"
#include "test.h"
#include "uart_non_blocking.h"

#define BUFFER_SIZE 32

void fire_command(const char *buf, size_t blen, UARTNB &uart) {
  if (blen == 0)
    return;
  if (strncmp(buf, "q", 1) == 0) {
    exit();
  } else if (strncmp(buf, "p", 1) == 0) {
    int parent_tid = my_parent_tid();
    uart.printf("My parent tid is %d\n\r", parent_tid);
  } else if (strncmp(buf, "m", 1) == 0) {
    int tid = my_tid();
    uart.printf("My tid is %d\n\r", tid);
  } else if (strncmp(buf, "y", 1) == 0) {
    yield();
    uart.puts("Yielded\n\r");
  } else if (strncmp(buf, "c", 1) == 0) {
    int tid = create(3, shell);
    uart.printf("Created new shell %u", tid);
  } else if (strncmp(buf, "t k1", 4) == 0) {
    int tid = create(2, test_k1);
    uart.printf("Created tid %u", tid);
  } else {
    uart.puts("Unknown command. Available: q (quit), p (parent tid), "
              "m (my tid), y (yield), c (create)\n\r");
  }
}

void shell() {
  char buf[BUFFER_SIZE];
  size_t buf_n = 0;
  UARTNB uart(CONSOLE);
  uart.puts("COMMANDS: q (quit) p (parent tid) m (my tid) y (yield) c "
            "(create)\n\r> ");
  while (1) {
    if (uart.can_receive_io()) {
      char c = uart.getc();
      if (isprint(c) && buf_n < BUFFER_SIZE - 1) {
        buf[buf_n++] = c;
        uart.putc(c);
      } else if ((c == 0x08 || c == 0x7f) && buf_n > 0) { // backspace
        uart.puts("\b \b"); // move back, print space, move back again
        --buf_n;
      } else if (c == '\r') { // enter
        buf[buf_n] = '\0';
        uart.puts("\n\r");
        fire_command(buf, buf_n, uart);
        uart.puts("> ");
        buf_n = 0;
      }
    } else {
      yield();
    }
    uart.send_io();
  }
}