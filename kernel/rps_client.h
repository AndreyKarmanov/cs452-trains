#pragma once

#include <optional>

#include "message.h"

class RPSClient {
  int rps_server_tid;

public:
  RPSClient();
  int tx_tid;

  std::optional<Message> signup();
  std::optional<Message> play(RPS::PlayMessage::Choice choice);
  std::optional<Message> quit();
};
