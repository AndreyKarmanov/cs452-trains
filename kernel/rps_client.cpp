#include "rps_client.h"
#include "debug.h"
#include "message.h"
#include "rps_server.h"
#include "syscall.h"
#include "uart.h"

RPSClient::RPSClient() {
  rps_server_tid = WhoIs(RPS_SERVER_NAME);
  _assert(rps_server_tid >= 0, "RPS server not found");
}

std::optional<Message> RPSClient::signup() {
  uart_printf(CONSOLE, "RPS client %d: signup\r\n", my_tid());
  Message msg{};
  msg.type = MessageType::RPS_SIGNUP;
  Message reply_msg{};
  int len = send(rps_server_tid, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg))) {
    uart_printf(CONSOLE, "RPS client %d: signup -> failed\r\n", my_tid());
    return std::nullopt;
  }
  if (reply_msg.type != MessageType::RPS_PLAY_READY) {
    uart_printf(CONSOLE, "RPS client %d: signup -> error\r\n", my_tid());
    if (reply_msg.type == MessageType::ERROR) {
      return reply_msg;
    }
    return std::nullopt;
  }
  return reply_msg;
}

std::optional<Message> RPSClient::play(RPS::PlayMessage::Choice choice) {
  uart_printf(CONSOLE, "RPS client %d: play %s\r\n", my_tid(),
              RPS::choice_str(choice));
  Message msg{};
  msg.type                 = MessageType::RPS_PLAY;
  msg.data.rps_play.choice = choice;
  Message reply_msg{};
  int len = send(rps_server_tid, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg))) {
    uart_printf(CONSOLE, "RPS client %d: play %s -> failed\r\n", my_tid(),
                RPS::choice_str(choice));
    return std::nullopt;
  }
  if (reply_msg.type != MessageType::RPS_PLAY_RESULT) {
    uart_printf(CONSOLE, "RPS client %d: play %s -> error\r\n", my_tid(),
                RPS::choice_str(choice));
    if (reply_msg.type == MessageType::ERROR) {
      return reply_msg;
    }
    return std::nullopt;
  }
  return reply_msg;
}

std::optional<Message> RPSClient::quit() {
  uart_printf(CONSOLE, "RPS client %d: quit\r\n", my_tid());
  Message msg{};
  msg.type = MessageType::RPS_QUIT;
  Message reply_msg{};
  int len = send(rps_server_tid, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg))) {
    uart_printf(CONSOLE, "RPS client %d: quit -> failed\r\n", my_tid());
    return std::nullopt;
  }
  if (reply_msg.type != MessageType::RPS_QUIT_ACK) {
    uart_printf(CONSOLE, "RPS client %d: quit -> error\r\n", my_tid());
    if (reply_msg.type == MessageType::ERROR) {
      return reply_msg;
    }
    return std::nullopt;
  }
  return reply_msg;
}

void test_rps_1_client_1() {
  RPSClient client;

  // test case 1: tie
  client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");

  // test case 2: win
  result = client.play(RPS::PlayMessage::Choice::PAPER);
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::WIN,
          "expected win");

  client.quit();
}

void test_rps_1_client_2() {
  RPSClient client;

  // test case 1: tie
  client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");

  // test case 2: lose
  result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::LOSE,
          "expected lose");
  client.quit();
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
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");
  client.quit();
}

void test_rps_2_client_2() {
  RPSClient client;

  // test case 2: partner quits
  client.signup();
  auto result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::PLAYER_QUIT,
          "expected player quit");

  // test case 2.1: cannot play if partner has quit
  auto result2 = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result2->type == MessageType::ERROR, "expected error");

  // test case 2.2: should be able to signup with new partner if partner has
  // quit
  client.signup();
  result = client.play(RPS::PlayMessage::Choice::ROCK);
  _assert(result->data.rps_play_result.result ==
              RPS::PlayResultMessage::Result::TIE,
          "expected tie");
  client.quit();
}

void test_rps_3_client_1() {
  RPSClient client;

  // test case 3: both users quit
  client.signup();
  auto result = client.quit();
  _assert(result->type == MessageType::RPS_QUIT_ACK,
          "Expected a response from quit");
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
}

void basic_scissors() {
  RPSClient client;
  client.signup();
  client.play(RPS::PlayMessage::Choice::SCISSORS);
  client.quit();
}

void basic_paper() {
  RPSClient client;
  client.signup();
  client.play(RPS::PlayMessage::Choice::PAPER);
  client.quit();
}

void basic_rock() {
  RPSClient client;
  client.signup();
  client.play(RPS::PlayMessage::Choice::ROCK);
  client.quit();
}

void test_rps_4() {
  // test if game allocator cycles games
  for (size_t i = 0; i < RPS_SERVER_MAX_GAMES + 5; i++) {
    auto tid1 = create(0, basic_scissors);
    auto tid2 = create(0, basic_paper);
    await_task(tid1);
    await_task(tid2);
  }
}

void test_rps_5() {
  auto tid1 = create(0, basic_rock);
  auto tid2 = create(0, basic_paper);
  auto tid3 = create(0, basic_scissors);
  auto tid4 = create(0, basic_rock);
  auto tid5 = create(0, basic_paper);
  auto tid6 = create(0, basic_scissors);
  auto tid7 = create(0, basic_paper);
  auto tid8 = create(0, basic_paper);

  await_task(tid1);
  await_task(tid2);
  await_task(tid3);
  await_task(tid4);
  await_task(tid5);
  await_task(tid6);
  await_task(tid7);
  await_task(tid8);
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
  test_rps_4();

  // test 5
  uart_printf(CONSOLE,
              "\n\r==============================================\n\r");
  uart_printf(CONSOLE, "Running RPS Test 5 (concurrent games)\n");
  uart_printf(CONSOLE, "==============================================\n\n");
  test_rps_5();

  uart_printf(CONSOLE, "\n\n All tests completed! \n\r");
}
