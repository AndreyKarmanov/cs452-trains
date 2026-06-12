#pragma once

#include "allocator.h"
#include "message.h"
#include "name_server.h"
#include <optional>

constexpr static char RPS_SERVER_NAME[]      = "RPS_SERVER";
constexpr static size_t RPS_SERVER_MAX_GAMES = 32;

class RPSServer {
  struct Game {
    enum class GameState {
      WaitingForBothPlayers,
      WaitingForPlayer1,
      WaitingForPlayer2,
      Finished,
      PartnerHasQuit,
    } game_state;
    int player1_tid;
    int player2_tid;
    RPS::PlayMessage::Choice player1_choice;
    RPS::PlayMessage::Choice player2_choice;
    int game_index;
  };

  std::optional<int> waiting = std::nullopt;

  Game games[RPS_SERVER_MAX_GAMES];
  Allocator<RPS_SERVER_MAX_GAMES> game_index_allocator;
  int player_to_game_ptr[RPS_SERVER_MAX_GAMES][2];

  std::optional<int> find_game_index_for_player(int tid);

public:
  RPSServer() {
    // register with name server
    RegisterAs(RPS_SERVER_NAME);

    // initialize players to game ptrs to -1 (no players)
    for (size_t i = 0; i < RPS_SERVER_MAX_GAMES; ++i) {
      player_to_game_ptr[i][0] = -1;
      player_to_game_ptr[i][1] = -1;
    }
  }

  void run();
};

void rps_server_task();
