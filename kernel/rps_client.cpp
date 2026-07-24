#include "rps_client.h"
#include "debug.h"
#include "io_helpers.h"
#include "message.h"
#include "rps_server.h"
#include "syscall.h"
#include "uart_tx_server.h"

namespace RPS {

  inline const char *choice_str(PlayMsg::Choice choice) {
    switch (choice) {
    case PlayMsg::Choice::ROCK:
      return "rock";
    case PlayMsg::Choice::PAPER:
      return "paper";
    case PlayMsg::Choice::SCISSORS:
      return "scissors";
    }
    return "?";
  }

  inline const char *result_str(PlayResultMsg::Result result) {
    switch (result) {
    case PlayResultMsg::Result::WIN:
      return "win";
    case PlayResultMsg::Result::LOSE:
      return "lose";
    case PlayResultMsg::Result::TIE:
      return "tie";
    case PlayResultMsg::Result::PLAYER_QUIT:
      return "partner quit";
    }
    return "?";
  }
} // namespace RPS

RPSClient::RPSClient() {
  tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  rps_server_tid = WhoIs(RPSServer<>::NAME);
  _assert(rps_server_tid >= 0, "RPS server not found");
}

std::optional<Message> RPSClient::signup() {
  Printf(tx_tid, "RPS client %d: signup\n\r", my_tid());
  auto reply_msg = send<RPS::PlayReadyMsg>(rps_server_tid, RPS::SetupMsg{});
  if (!reply_msg.has_value()) {
    Printf(tx_tid, "RPS client %d: signup -> failed\n\r", my_tid());
    return std::nullopt;
  }
  return *reply_msg;
}

std::optional<Message> RPSClient::play(RPS::PlayMsg::Choice choice) {
  Printf(tx_tid, "RPS client %d: play %s\n\r", my_tid(),
         RPS::choice_str(choice));
  auto reply_msg =
      send<RPS::PlayResultMsg>(rps_server_tid, RPS::PlayMsg{.choice = choice});
  if (!reply_msg.has_value()) {
    Printf(tx_tid, "RPS client %d: play %s -> failed\n\r", my_tid(),
           RPS::choice_str(choice));
    return std::nullopt;
  }
  return *reply_msg;
}

std::optional<Message> RPSClient::quit() {
  Printf(tx_tid, "RPS client %d: quit\n\r", my_tid());
  auto reply_msg = send<RPS::QuitAckMsg>(rps_server_tid, RPS::QuitMsg{});
  if (!reply_msg.has_value()) {
    Printf(tx_tid, "RPS client %d: quit -> failed\n\r", my_tid());
    return std::nullopt;
  }
  return *reply_msg;
}
