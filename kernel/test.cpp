#include "test.h"
#include "name_server.h"
#include "syscall.h"
#include "time.h"
#include "uart.h"

#define N_ITERATIONS 100000

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

void test_timer_task() {
  uart_puts(CONSOLE, "test timer task start\n\r");
  auto tid_a = create(3, test_timer_a_task);
  uart_printf(CONSOLE, "tid a: %d ", tid_a);
  auto tid_b = create(3, test_timer_b_task);
  uart_printf(CONSOLE, "tid b: %d\n\r", tid_b);
  exit();
}

void test_timer_a_task() {
  RegisterAs("test_timer_a_task");
  int other = WhoIs("test_timer_b_task");
  while (other == -1) {
    other = WhoIs("test_timer_b_task");
  }

  uart_puts(CONSOLE, "test timer A task start\n\r");
  uart_puts(CONSOLE, "A sending\n\r");

  char msg_buf_4[4];
  char reply_buf_4[4];
  auto start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_4, sizeof(msg_buf_4), reply_buf_4, sizeof(reply_buf_4));
  }
  auto delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "4 Byte Messages: %u.%uus\n\r", delta / 10, delta % 10);

  char msg_buf_64[64];
  char reply_buf_64[64];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_64, sizeof(msg_buf_64), reply_buf_64,
         sizeof(reply_buf_64));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "64 Byte Messages: %u.%uus\n\r", delta / 10, delta % 10);

  char msg_buf_256[256];
  char reply_buf_256[256];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_256, sizeof(msg_buf_256), reply_buf_256,
         sizeof(reply_buf_256));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "256 Byte Messages: %u.%uus\n\r", delta / 10,
              delta % 10);

  uart_puts(CONSOLE, "A recieving\n\r");

  char rcv_buf_4[4];
  char reply_msg_buf_4[4];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_4, sizeof(rcv_buf_4));
    reply(other, reply_msg_buf_4, sizeof(reply_msg_buf_4));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "4 Byte Messages: %u.%uus\n\r", delta / 10, delta % 10);

  char rcv_buf_64[64]{1};
  char reply_msg_buf_64[64]{1};
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_64, sizeof(rcv_buf_64));
    reply(other, reply_msg_buf_64, sizeof(reply_msg_buf_64));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "64 Byte Messages: %u.%uus\n\r", delta / 10, delta % 10);

  char rcv_buf_256[256];
  char reply_msg_buf_256[256];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_256, sizeof(rcv_buf_256));
    reply(other, reply_msg_buf_256, sizeof(reply_msg_buf_256));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "256 Byte Messages: %u.%uus\n\r", delta / 10,
              delta % 10);

  exit();
}

void test_timer_b_task() {
  RegisterAs("test_timer_b_task");
  int other = WhoIs("test_timer_a_task");
  while (other == -1) {
    other = WhoIs("test_timer_a_task");
  }

  uart_puts(CONSOLE, "test timer B task start\n\r");
  uart_puts(CONSOLE, "B recieving\n\r");

  char rcv_buf_4[4];
  char reply_msg_buf_4[4];
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_4, sizeof(rcv_buf_4));
    reply(other, reply_msg_buf_4, sizeof(reply_msg_buf_4));
  }
  char rcv_buf_64[64];
  char reply_msg_buf_64[64];
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_64, sizeof(rcv_buf_64));
    reply(other, reply_msg_buf_64, sizeof(reply_msg_buf_64));
  }

  char rcv_buf_256[256];
  char reply_msg_buf_256[256];
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_256, sizeof(rcv_buf_256));
    reply(other, reply_msg_buf_256, sizeof(reply_msg_buf_256));
  }

  uart_puts(CONSOLE, "B sending \n\r");

  char msg_buf_4[4];
  char reply_buf_4[4];
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_4, sizeof(msg_buf_4), reply_buf_4, sizeof(reply_buf_4));
  }

  char msg_buf_64[64];
  char reply_buf_64[64];
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_64, sizeof(msg_buf_64), reply_buf_64,
         sizeof(reply_buf_64));
  }

  char msg_buf_256[256];
  char reply_buf_256[256];
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_256, sizeof(msg_buf_256), reply_buf_256,
         sizeof(reply_buf_256));
  }

  exit();
}
