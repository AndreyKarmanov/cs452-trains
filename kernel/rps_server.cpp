#include "rps_server.h"
#include "debug.h"
#include "io_helpers.h"
#include "message.h"
#include "syscall.h"

void RPSServer::handle(const int sender_tid, const RPS::SetupMsg &) {
  if (find_game_index_for_player(sender_tid).has_value() ||
      (waiting.has_value() && waiting.value() == sender_tid)) {
    reply_with_error(sender_tid);
    return;
  }

  if (!waiting.has_value()) {
    waiting.emplace(sender_tid);
    return;
  }

  auto p1 = waiting.value();
  auto p2 = sender_tid;
  _assert(p1 != p2, "STARTED GAME WITH SELF");

  auto game_index = game_index_allocator.allocate();
  if (!game_index.has_value()) {
    reply_with_error(p2);
    return;
  }

  auto game         = &games[game_index.value()];
  game->game_state  = Game::GameState::WaitingForBothPlayers;
  game->player1_tid = p1;
  game->player2_tid = p2;
  game->game_index  = game_index.value();

  player_to_game_ptr[game_index.value()][0] = p1;
  player_to_game_ptr[game_index.value()][1] = p2;
  waiting.reset();

  Printf(tx_tid, "RPS server: Game started between player %d and player %d\r\n",
         p1, p2);

  reply(p1, RPS::PlayReadyMsg{});
  reply(p2, RPS::PlayReadyMsg{});
}

void RPSServer::handle(const int sender_tid, const RPS::PlayMsg &arg) {
  auto game_index = find_game_index_for_player(sender_tid);
  if (!game_index.has_value()) {
    reply_with_error(sender_tid);
    return;
  }

  auto game         = &games[game_index.value()];
  bool is_player1   = game->player1_tid == sender_tid;
  int partner_index = is_player1 ? 1 : 0;
  int partner_tid   = is_player1 ? game->player2_tid : game->player1_tid;

  if (game->game_state == Game::GameState::PartnerHasQuit) {
    game_index_allocator.free(game_index.value());
    player_to_game_ptr[game_index.value()][1 - partner_index] = -1;
    Printf(tx_tid, "RPS server: Game ended between player %d and player %d\r\n",
           sender_tid, partner_tid);
    reply(sender_tid, RPS::PlayResultMsg{
                          .result = RPS::PlayResultMsg::Result::PLAYER_QUIT});
    return;
  }

  using Choice = RPS::PlayMsg::Choice;
  auto choice  = arg.choice;
  if (choice != Choice::ROCK && choice != Choice::PAPER &&
      choice != Choice::SCISSORS) {
    reply_with_error(sender_tid);
    return;
  }

  if (is_player1) {
    if (game->game_state == Game::GameState::WaitingForBothPlayers) {
      game->player1_choice = choice;
      game->game_state     = Game::GameState::WaitingForPlayer2;
    } else if (game->game_state == Game::GameState::WaitingForPlayer1) {
      game->player1_choice = choice;
      game->game_state     = Game::GameState::Finished;
    } else {
      reply_with_error(sender_tid);
      return;
    }
  } else {
    if (game->game_state == Game::GameState::WaitingForBothPlayers) {
      game->player2_choice = choice;
      game->game_state     = Game::GameState::WaitingForPlayer1;
    } else if (game->game_state == Game::GameState::WaitingForPlayer2) {
      game->player2_choice = choice;
      game->game_state     = Game::GameState::Finished;
    } else {
      reply_with_error(sender_tid);
      return;
    }
  }

  using Result = RPS::PlayResultMsg::Result;
  if (game->game_state == Game::GameState::Finished) {
    auto p1_result = game->player1_choice == game->player2_choice ? Result::TIE
                                                                  : Result::WIN;
    auto p2_result = game->player1_choice == game->player2_choice
                         ? Result::TIE
                         : Result::LOSE;

    if ((game->player1_choice == Choice::ROCK &&
         game->player2_choice == Choice::PAPER) ||
        (game->player1_choice == Choice::PAPER &&
         game->player2_choice == Choice::SCISSORS) ||
        (game->player1_choice == Choice::SCISSORS &&
         game->player2_choice == Choice::ROCK)) {
      p1_result = Result::LOSE;
      p2_result = Result::WIN;
    }

    Printf(tx_tid, "RPS server: Game result: player %d %s, player %d %s\r\n",
           game->player1_tid, RPS::result_str(p1_result), game->player2_tid,
           RPS::result_str(p2_result));

    game->game_state = Game::GameState::WaitingForBothPlayers;
    reply(game->player1_tid, RPS::PlayResultMsg{.result = p1_result});
    reply(game->player2_tid, RPS::PlayResultMsg{.result = p2_result});
  }
}

void RPSServer::handle(const int sender_tid, const RPS::QuitMsg &) {
  auto game_index = find_game_index_for_player(sender_tid);
  if (!game_index.has_value()) {
    reply_with_error(sender_tid);
    return;
  }

  auto game              = &games[game_index.value()];
  bool sender_is_player1 = game->player1_tid == sender_tid;
  auto partner_tid = sender_is_player1 ? game->player2_tid : game->player1_tid;

  bool quit_user        = false;
  bool deallocate_game  = false;
  bool reply_to_partner = false;
  switch (game->game_state) {
  case Game::GameState::WaitingForBothPlayers:
    quit_user = true;
    break;
  case Game::GameState::WaitingForPlayer1:
    if (sender_is_player1) {
      deallocate_game  = true;
      reply_to_partner = true;
    }
    break;
  case Game::GameState::WaitingForPlayer2:
    if (!sender_is_player1) {
      deallocate_game  = true;
      reply_to_partner = true;
    }
    break;
  case Game::GameState::PartnerHasQuit:
    deallocate_game = true;
    break;
  default:
    reply_with_error(sender_tid);
    return;
  }

  if (quit_user) {
    player_to_game_ptr[game->game_index][sender_is_player1 ? 0 : 1] = -1;
    game->game_state = Game::GameState::PartnerHasQuit;
  }

  if (deallocate_game) {
    player_to_game_ptr[game->game_index][0] = -1;
    player_to_game_ptr[game->game_index][1] = -1;
    game_index_allocator.free(game->game_index);
    Printf(tx_tid, "RPS server: Game ended between player %d and player %d\r\n",
           sender_tid, partner_tid);
  }

  if (reply_to_partner) {
    reply(partner_tid, RPS::QuitAckMsg{});
  }
  reply(sender_tid, RPS::QuitAckMsg{});
}

void RPSServer::run() {
  int sender_tid;
  Message msg{};
  receive(&sender_tid, msg);

  std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
}

std::optional<int> RPSServer::find_game_index_for_player(int tid) {
  for (size_t i = 0; i < RPS_SERVER_MAX_GAMES; ++i) {
    if (player_to_game_ptr[i][0] == tid) {
      return i;
    }
    if (player_to_game_ptr[i][1] == tid) {
      return i;
    }
  }
  return std::nullopt;
}

void rps_server_task() {
  static RPSServer server{};
  for (;;) {
    server.run();
  }
}
