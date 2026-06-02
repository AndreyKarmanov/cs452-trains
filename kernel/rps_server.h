#pragma once

#include "basic_map.h"
#include <optional>

class RPSServer {
  static constexpr size_t MAX_GAMES = 32;
  std::optional<int> waiting_tid    = std::nullopt;
  BasicMap<int, int, MAX_GAMES * 2> games;

public:
  void run();
};

void rps_server_task();