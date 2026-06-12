#pragma once

#include "message.h"
#include <optional>

class RPSClient {
  int rps_server_tid;

public:
  RPSClient();

  std::optional<Message> signup();
  std::optional<Message> play(RPS::PlayMessage::Choice choice);
  std::optional<Message> quit();
};
