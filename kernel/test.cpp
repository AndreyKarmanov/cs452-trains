#include "test.h"
#include "name_server.h"
#include "syscall.h"
#include "time.h"
#include "uart.h"

#define N_ITERATIONS 100000

void test_k1() {
  debug_puts(CONSOLE, "\n\rTesting K1\n\r");
  create(1, test_k1_child);
  debug_puts(CONSOLE, "Created child 1\n\r");
  create(1, test_k1_child);
  debug_puts(CONSOLE, "Created child 2\n\r");
  create(3, test_k1_child);
  debug_puts(CONSOLE, "Created child 3\n\r");
  create(3, test_k1_child);
  debug_puts(CONSOLE, "Created child 4\n\r");

  // end
  debug_puts(CONSOLE, "FirstUserTask: Exiting\n\r");
}

void test_k1_child() {
  debug_printf(CONSOLE, "Child tid: %d, Parent tid: %d\n\r", my_tid(),
               my_parent_tid());
  yield();
  debug_printf(CONSOLE, "Child tid: %d, Parent tid: %d\n\r", my_tid(),
               my_parent_tid());
}

void test_timer_a_task() {
  RegisterAs("test_timer_a_task");
  int other = WhoIs("test_timer_b_task");

  debug_puts(CONSOLE, "A sending\n\r");

  char msg_buf_4[4];
  char reply_buf_4[4];
  auto start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_4, sizeof(msg_buf_4), reply_buf_4, sizeof(reply_buf_4));
  }
  auto delta = (time_get() - start) / (N_ITERATIONS / 10);
  debug_puts(CONSOLE, "4 byte / 64 byte / 256 byte (us)\n\r");
  debug_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char msg_buf_64[64];
  char reply_buf_64[64];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_64, sizeof(msg_buf_64), reply_buf_64,
         sizeof(reply_buf_64));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  debug_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char msg_buf_256[256];
  char reply_buf_256[256];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_256, sizeof(msg_buf_256), reply_buf_256,
         sizeof(reply_buf_256));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  debug_printf(CONSOLE, "%u.%uus\n\r", delta / 10, delta % 10);

  debug_puts(CONSOLE, "A recieving\n\r");

  char rcv_buf_4[4];
  char reply_msg_buf_4[4];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_4, sizeof(rcv_buf_4));
    reply(other, reply_msg_buf_4, sizeof(reply_msg_buf_4));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  debug_puts(CONSOLE, "4 byte / 64 byte / 256 byte (us)\n\r");
  debug_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char rcv_buf_64[64]{1};
  char reply_msg_buf_64[64]{1};
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_64, sizeof(rcv_buf_64));
    reply(other, reply_msg_buf_64, sizeof(reply_msg_buf_64));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  debug_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char rcv_buf_256[256];
  char reply_msg_buf_256[256];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_256, sizeof(rcv_buf_256));
    reply(other, reply_msg_buf_256, sizeof(reply_msg_buf_256));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  debug_printf(CONSOLE, "%u.%uus \n\r", delta / 10, delta % 10);
}

void test_timer_b_task() {
  RegisterAs("test_timer_b_task");
  int other = WhoIs("test_timer_a_task");

  debug_puts(CONSOLE, "B recieving\n\r");

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

  debug_puts(CONSOLE, "B sending \n\r");

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
}

void test_timer_task() {
  debug_printf(CONSOLE, "test timer task start\n\r");
  auto tid_a = create(1, test_timer_a_task);
  debug_printf(CONSOLE, "tid a: %d ", tid_a);
  auto tid_b = create(1, test_timer_b_task);
  debug_printf(CONSOLE, "tid b: %d\n\r", tid_b);
}

void test_await_event_task() {
  await_event(Event::DELAY_5S);
  debug_puts(CONSOLE, "5 second delay task");
}

void test_can_interrupt_task() {
  CANFRAME frame;
  for (size_t i = 0; i < 5; ++i) {
    await_event(Event::CAN_RX_IRQ);
    rx_can(frame);
    debug_printf(CONSOLE, "Received CAN frame!\n\r");
  }
}