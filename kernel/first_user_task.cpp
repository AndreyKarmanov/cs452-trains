#include "first_user_task.h"
#include "debug.h"
#include "name_server.h"
#include "rps_client.h"
#include "rps_server.h"
#include "syscall.h"
#include "test.h"
#include "uart.h"

void test_rps_1_client_1() {
  RPSClient client;

  // test case 1: tie
  auto ready  = client.signup();
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
  auto ready  = client.signup();
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
  auto ready  = client.signup();
  auto result = client.quit();
  _assert(result->type == MessageType::RPS_QUIT_ACK,
          "Expected a response from quit");

  // test 2.2: former partner can sign up with new partner
  auto ready2 = client.signup();
  result      = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");
  client.quit();
  exit();
}

void test_rps_2_client_2() {
  RPSClient client;

  // test case 2: partner quits
  auto ready  = client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::PLAYER_QUIT,
          "expected player quit");

  // test case 2.1: cannot play if partner has quit
  auto result2 = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result2->type == MessageType::ERROR, "expected error");

  // test case 2.2: should be able to signup with new partner if partner has
  // quit
  auto ready3 = client.signup();
  result      = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->payload.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");
  client.quit();
  exit();
}

void test_rps_3_client_1() {
  RPSClient client;

  // test case 3: both users quit
  auto ready  = client.signup();
  auto result = client.quit();
  _assert(result->type == MessageType::RPS_QUIT_ACK,
          "Expected a response from quit");
  exit();
}

void test_rps_3_client_2() {
  RPSClient client;

  // test case 3: both users quit
  auto ready  = client.signup();
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
  auto ready  = client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::SCISSORS);
  client.quit();
  exit();
}

void basic_paper() {
  RPSClient client;
  auto ready  = client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::PAPER);
  client.quit();
  exit();
}

void basic_rock() {
  RPSClient client;
  auto ready  = client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  client.quit();
  exit();
}

void test_rps_4() {
  // test if game allocator cycles games
  for (int i = 0; i < RPS_SERVER_MAX_GAMES + 5; i++) {
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

void rps_client_task() {
  // test 1: test normal cases
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 1 (normal cases)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  create(2, test_rps_1_client_1);
  create(2, test_rps_1_client_2);

  // test 2
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 2 (1 player quits)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  create(2, test_rps_2_client_1);
  create(2, test_rps_2_client_2);

  // test 3
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 3 (both players quit)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  create(2, test_rps_3_client_1);
  create(2, test_rps_3_client_2);

  // test 4
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 4 (lots of games)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  create(1, test_rps_4);

  // test 5
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 5 (concurrent games)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  create(2, test_rps_5);

  uart_printf(CONSOLE, "\n\n All tests completed! \n\r");
  exit();
}

void first_user_task() {
  create(2, name_server_task);
  uart_printf(CONSOLE, "Created name server\n");

  // RPS
  create(2, rps_server_task);
  uart_printf(CONSOLE, "Created RPS server\n");
  create(0, rps_client_task);

  // Timer
  create(2, test_timer_task);
  uart_printf(CONSOLE, "Created timer task\n");

  // Shell
  // create(3, shell_task);

  exit();
}
