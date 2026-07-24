#include "rps_client.h"
#include "debug.h"
#include "io_helpers.h"
#include "message.h"
#include "rps_server.h"
#include "syscall.h"
#include "uart_tx_server.h"

RPSClient::RPSClient() {
  tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  rps_server_tid = WhoIs(RPSServer<>::NAME);
  _assert(rps_server_tid >= 0, "RPS server not found");
}

std::optional<Message> RPSClient::signup() {
  Puts(tx_tid, "RPS client ", my_tid(), ": signup\n\r");
  auto reply_msg = send<RPS::PlayReadyMsg>(rps_server_tid, RPS::SetupMsg{});
  if (!reply_msg.has_value()) {
    Puts(tx_tid, "RPS client ", my_tid(), ": signup -> failed\n\r");
    return std::nullopt;
  }
  return *reply_msg;
}

std::optional<Message> RPSClient::play(RPS::PlayMsg::Choice choice) {
  Puts(tx_tid, "RPS client ", my_tid(), ": play ", RPS::choice_str(choice),
       "\n\r");
  auto reply_msg =
      send<RPS::PlayResultMsg>(rps_server_tid, RPS::PlayMsg{.choice = choice});
  if (!reply_msg.has_value()) {
    Puts(tx_tid, "RPS client ", my_tid(), ": play ", RPS::choice_str(choice),
         " -> failed\n\r");
    return std::nullopt;
  }
  return *reply_msg;
}

std::optional<Message> RPSClient::quit() {
  Puts(tx_tid, "RPS client ", my_tid(), ": quit\n\r");
  auto reply_msg = send<RPS::QuitAckMsg>(rps_server_tid, RPS::QuitMsg{});
  if (!reply_msg.has_value()) {
    Puts(tx_tid, "RPS client ", my_tid(), ": quit -> failed\n\r");
    return std::nullopt;
  }
  return *reply_msg;
}
