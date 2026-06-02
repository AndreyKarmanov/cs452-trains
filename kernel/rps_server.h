#pragma once

#include <optional>

#include "basic_map.h"
#include "message.h"

class RPSServer {
  static constexpr size_t MAX_GAMES = 32;
  std::optional<int> waiting        = std::nullopt;

  BasicMap<int, int, MAX_GAMES * 2> partners;
  BasicMap<int, RPS::PlayMessage::Choice, MAX_GAMES * 2> choices;

public:
  void run();
};

void rps_server_task();