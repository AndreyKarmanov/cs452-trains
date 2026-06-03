#pragma once

#include <optional>

#include "allocator.h"
#include "message.h"

class RPSServer {
  struct Game {
    enum class GameState {
      WaitingForBothPlayers,
      WaitingForPlayer1,
      WaitingForPlayer2,
      Finished
    } game_state;
    int player1_tid;
    int player2_tid;
    RPS::PlayMessage::Choice player1_choice;
    RPS::PlayMessage::Choice player2_choice;
    int game_index;
  };

  static constexpr size_t MAX_GAMES = 32;
  std::optional<int> waiting        = std::nullopt;

  Game games[MAX_GAMES];
  Allocator<MAX_GAMES> game_index_allocator;
  // BasicMap<int, Game *, MAX_GAMES * 2> player_to_game_ptr_map;
  int player_to_game_ptr[MAX_GAMES][2];

  std::optional<int> find_game_index_for_player(int tid);

public:
  RPSServer() {
    for (size_t i = 0; i < MAX_GAMES; ++i) {
      player_to_game_ptr[i][0] = -1;
      player_to_game_ptr[i][1] = -1;
    }
  }

  void run();
};

void rps_server_task();
