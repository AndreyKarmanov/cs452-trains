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
  msg.type                    = MessageType::RPS_PLAY;
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
