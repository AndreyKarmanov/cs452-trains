#include "test.h"
#include "syscall.h"
#include "uart.h"

void test_k1() {
  uart_puts(CONSOLE, "\n\rTesting K1\n\r");
  create(2, test_k1_child);
  uart_puts(CONSOLE, "Created child 1\n\r");
  create(2, test_k1_child);
  uart_puts(CONSOLE, "Created child 2\n\r");
  create(1, test_k1_child);
  uart_puts(CONSOLE, "Created child 3\n\r");
  create(1, test_k1_child);
  uart_puts(CONSOLE, "Created child 4\n\r");

  // end
  uart_puts(CONSOLE, "FirstUserTask: Exiting\n\r");
  exit();
}

void test_k1_child() {
  uart_printf(CONSOLE, "Child tid: %d, Parent tid: %d\n\r", my_tid(),
              my_parent_tid());
  yield();
  uart_printf(CONSOLE, "Child tid: %d, Parent tid: %d\n\r", my_tid(),
              my_parent_tid());
  exit();
}