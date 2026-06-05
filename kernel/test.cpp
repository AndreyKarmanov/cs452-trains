#include "test.h"
#include "debug.h"
#include "message.h"
#include "name_server.h"
#include "rps_client.h"
#include "rps_server.h"
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

void test_timer_a_task() {
  RegisterAs("test_timer_a_task");
  int other = WhoIs("test_timer_b_task");
  while (other == -1) {
    other = WhoIs("test_timer_b_task");
  }

  uart_puts(CONSOLE, "A sending\n\r");

  char msg_buf_4[4];
  char reply_buf_4[4];
  auto start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_4, sizeof(msg_buf_4), reply_buf_4, sizeof(reply_buf_4));
  }
  auto delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_puts(CONSOLE, "4 byte / 64 byte / 256 byte (us)\n\r");
  uart_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char msg_buf_64[64];
  char reply_buf_64[64];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_64, sizeof(msg_buf_64), reply_buf_64,
         sizeof(reply_buf_64));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char msg_buf_256[256];
  char reply_buf_256[256];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    send(other, msg_buf_256, sizeof(msg_buf_256), reply_buf_256,
         sizeof(reply_buf_256));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "%u.%uus\n\r", delta / 10, delta % 10);

  uart_puts(CONSOLE, "A recieving\n\r");

  char rcv_buf_4[4];
  char reply_msg_buf_4[4];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_4, sizeof(rcv_buf_4));
    reply(other, reply_msg_buf_4, sizeof(reply_msg_buf_4));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_puts(CONSOLE, "4 byte / 64 byte / 256 byte (us)\n\r");
  uart_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char rcv_buf_64[64]{1};
  char reply_msg_buf_64[64]{1};
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_64, sizeof(rcv_buf_64));
    reply(other, reply_msg_buf_64, sizeof(reply_msg_buf_64));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "%u.%uus ", delta / 10, delta % 10);

  char rcv_buf_256[256];
  char reply_msg_buf_256[256];
  start = time_get();
  for (size_t i = 0; i < N_ITERATIONS; ++i) {
    receive(&other, rcv_buf_256, sizeof(rcv_buf_256));
    reply(other, reply_msg_buf_256, sizeof(reply_msg_buf_256));
  }
  delta = (time_get() - start) / (N_ITERATIONS / 10);
  uart_printf(CONSOLE, "%u.%uus \n\r", delta / 10, delta % 10);

  exit();
}

void test_timer_b_task() {
  RegisterAs("test_timer_b_task");
  int other = WhoIs("test_timer_a_task");
  while (other == -1) {
    other = WhoIs("test_timer_a_task");
  }

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

void test_timer_task() {
  uart_puts(CONSOLE, "test timer task start\n\r");
  auto tid_a = create(3, test_timer_a_task);
  uart_printf(CONSOLE, "tid a: %d ", tid_a);
  auto tid_b = create(3, test_timer_b_task);
  uart_printf(CONSOLE, "tid b: %d\n\r", tid_b);
  exit();
}

void test_rps_1_client_1() {
  RPSClient client;

  // test case 1: tie
  client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");

  // test case 2: win
  result = client.play(RPS::PlayMessage::Choice::PAPER);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::WIN,
          "expected win");

  client.quit();
  exit();
}

void test_rps_1_client_2() {
  RPSClient client;

  // test case 1: tie
  client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");

  // test case 2: lose
  result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::LOSE,
          "expected lose");
  client.quit();
  exit();
}

void test_rps_2_client_1() {
  RPSClient client;

  // test case 2: user quits
  client.signup();
  auto result = client.quit();
  _assert(result->type == MessageType::RPS_QUIT_ACK,
          "Expected a response from quit");

  // test 2.2: former partner can sign up with new partner
  client.signup();
  result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");
  client.quit();
  exit();
}

void test_rps_2_client_2() {
  RPSClient client;

  // test case 2: partner quits
  client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::PLAYER_QUIT,
          "expected player quit");

  // test case 2.1: cannot play if partner has quit
  auto result2 = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result2->type == MessageType::ERROR, "expected error");

  // test case 2.2: should be able to signup with new partner if partner has
  // quit
  client.signup();
  result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");
  client.quit();
  exit();
}

void test_rps_3_client_1() {
  RPSClient client;

  // test case 3: both users quit
  client.signup();
  auto result = client.quit();
  _assert(result->type == MessageType::RPS_QUIT_ACK,
          "Expected a response from quit");
  exit();
}

void test_rps_3_client_2() {
  RPSClient client;

  // test case 3: both users quit
  client.signup();
  auto result = client.quit();
  _assert(result->type == MessageType::RPS_QUIT_ACK,
          "Expected a response from quit");

  // test case 3.1: cannot play if has quit
  auto result2 = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result2->type == MessageType::ERROR, "expected error");
  exit();
}

void basic_scissors() {
  RPSClient client;
  client.signup();
  client.play(RPS::PlayMessage::Choice::SCISSORS);
  client.quit();
  exit();
}

void basic_paper() {
  RPSClient client;
  client.signup();
  client.play(RPS::PlayMessage::Choice::PAPER);
  client.quit();
  exit();
}

void basic_rock() {
  RPSClient client;
  client.signup();
  client.play(RPS::PlayMessage::Choice::ROCK);
  client.quit();
  exit();
}

void test_rps_4() {
  // test if game allocator cycles games
  for (size_t i = 0; i < RPS_SERVER_MAX_GAMES + 5; i++) {
    create(2, basic_scissors);
    create(2, basic_paper);
    yield();
  }
  exit();
}

void test_rps_5() {
  create(1, basic_rock);
  create(1, basic_paper);
  create(1, basic_scissors);
  create(1, basic_rock);
  create(1, basic_paper);
  create(1, basic_scissors);
  create(1, basic_paper);
  create(1, basic_paper);
  exit();
}

void test_rps_task() {
  // test 1: test normal cases
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 1 (normal cases)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  auto tid1 = create(0, test_rps_1_client_1);
  auto tid2 = create(0, test_rps_1_client_2);
  await_task(tid1);
  await_task(tid2);

  // test 2
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 2 (1 player quits)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  auto tid3 = create(0, test_rps_2_client_1);
  auto tid4 = create(0, test_rps_2_client_2);
  await_task(tid3);
  await_task(tid4);

  // test 3
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 3 (both players quit)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  auto tid5 = create(0, test_rps_3_client_1);
  auto tid6 = create(0, test_rps_3_client_2);
  await_task(tid5);
  await_task(tid6);

  // test 4
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 4 (lots of games)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  auto tid7 = create(0, test_rps_4);
  await_task(tid7);

  // test 5
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 5 (concurrent games)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  auto tid8 = create(0, test_rps_5);
  await_task(tid8);

  uart_printf(CONSOLE, "\n\n All tests completed! \n\r");
  exit();
}
