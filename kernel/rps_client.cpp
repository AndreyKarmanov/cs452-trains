#include "rps_client.h"
#include "debug.h"
#include "io_helpers.h"
#include "message.h"
#include "rps_server.h"
#include "syscall.h"
#include "tx_server.h"

RPSClient::RPSClient() {
  tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  rps_server_tid = WhoIs(RPS_SERVER_NAME);
  _assert(rps_server_tid >= 0, "RPS server not found");
}

std::optional<Message> RPSClient::signup() {
  Printf(tx_tid, "RPS client %d: signup\r\n", my_tid());
  auto reply_msg = send<RPS::PlayReadyMsg>(rps_server_tid, RPS::SetupMsg{});
  if (!reply_msg.has_value()) {
    Printf(tx_tid, "RPS client %d: signup -> failed\r\n", my_tid());
    return std::nullopt;
  }
  return *reply_msg;
}

std::optional<Message> RPSClient::play(RPS::PlayMsg::Choice choice) {
  Printf(tx_tid, "RPS client %d: play %s\r\n", my_tid(),
         RPS::choice_str(choice));
  auto reply_msg =
      send<RPS::PlayResultMsg>(rps_server_tid, RPS::PlayMsg{.choice = choice});
  if (!reply_msg.has_value()) {
    Printf(tx_tid, "RPS client %d: play %s -> failed\r\n", my_tid(),
           RPS::choice_str(choice));
    return std::nullopt;
  }
  return *reply_msg;
}

std::optional<Message> RPSClient::quit() {
  Printf(tx_tid, "RPS client %d: quit\r\n", my_tid());
  auto reply_msg = send<RPS::QuitAckMsg>(rps_server_tid, RPS::QuitMsg{});
  if (!reply_msg.has_value()) {
    Printf(tx_tid, "RPS client %d: quit -> failed\r\n", my_tid());
    return std::nullopt;
  }
  return *reply_msg;
}
