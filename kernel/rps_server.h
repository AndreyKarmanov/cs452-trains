#pragma once

#include "allocator.h"
#include "message.h"
#include "name_server.h"
#include "uart_tx_server.h"

template <size_t MAX_GAMES = 32> class RPSServer {
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
    RPS::PlayMsg::Choice player1_choice;
    RPS::PlayMsg::Choice player2_choice;
    int game_index;
  };

  std::optional<int> waiting = std::nullopt;

  Game games[MAX_GAMES];
  Allocator<MAX_GAMES> game_index_allocator;
  int player_to_game_ptr[MAX_GAMES][2];
  int tx_tid;
  std::optional<int> find_game_index_for_player(int tid);

  void handle(const int tid, const RPS::SetupMsg &);
  void handle(const int tid, const RPS::PlayMsg &);
  void handle(const int tid, const RPS::QuitMsg &);
  template <class T> void handle(int tid, const T &) { reply_with_error(tid); }

public:
  constexpr static auto NAME = "RPS_SERVER";

  RPSServer() {
    // register with name server
    RegisterAs(NAME);

    tx_tid = WhoIs(UART_TX_Server::NAME);
    _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

    // initialize players to game ptrs to -1 (no players)
    for (size_t i = 0; i < MAX_GAMES; ++i) {
      player_to_game_ptr[i][0] = -1;
      player_to_game_ptr[i][1] = -1;
    }
  }

  void run();
};

void rps_server_task();
