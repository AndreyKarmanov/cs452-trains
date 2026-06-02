#include "test.h"
#include "name_server.h"
#include "syscall.h"
#include "uart.h"

void test_k1() {
  uart_puts(CONSOLE, "\n\rTesting K1\n\r");
  create(1, test_k1_child);
  uart_puts(CONSOLE, "Created child 1\n\r");
  create(1, test_k1_child);
  uart_puts(CONSOLE, "Created child 2\n\r");
  create(3, test_k1_child);
  uart_puts(CONSOLE, "Created child 3\n\r");
  create(3, test_k1_child);
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

void test_name_server() {
  int me = my_tid();
  int r  = RegisterAs("Clock");
  uart_printf(CONSOLE, "RegisterAs Clock: %d (expect 0)\n\r", r);
  int tid = WhoIs("Clock");
  uart_printf(CONSOLE, "WhoIs Clock: %d (expect %d)\n\r", tid, me);
  int r2 = RegisterAs("Test2");
  uart_printf(CONSOLE, "RegisterAs Test2: %d (expect 0)\n\r", r2);
  tid = WhoIs("Test2");
  uart_printf(CONSOLE, "WhoIs Test2: %d (expect %d)\n\r", tid, me);
  tid = WhoIs("Missing");
  uart_printf(CONSOLE, "WhoIs Missing: %d (expect -1)\n\r", tid);
  exit();
}

void name_server() {
  NameServer server;
  for (;;) {
    server.run();
    yield();
  }
}
