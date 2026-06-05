
#include <optional>

#include "debug.h"
#include "message.h"
#include "rps_server.h"
#include "syscall.h"
#include "uart.h"

void RPSServer::run() {
  int sender_tid;
  Message msg{};
  int len = receive(&sender_tid, (char *)&msg, sizeof(msg));

  if (len < static_cast<int>(sizeof(msg))) {
    reply_with_error(sender_tid);
    return;
  }

  switch (msg.type) {
  case MessageType::RPS_SIGNUP: {
    // check if player is already in a game or waiting to join a game
    if (find_game_index_for_player(sender_tid).has_value() ||
        (waiting.has_value() && waiting.value() == sender_tid)) {
      reply_with_error(sender_tid);
      break;
    }
    if (!waiting.has_value()) {
      // no one available, start to wait
      waiting.emplace(sender_tid);
    } else {
      auto p1 = waiting.value();
      auto p2 = sender_tid;

      _assert(p1 != p2, "STARTED GAME WITH SELF");

      // start new game
      auto game_index = game_index_allocator.allocate();
      if (!game_index.has_value()) {
        reply_with_error(p2);
        break;
      }
      auto game         = &games[game_index.value()];
      game->game_state  = Game::GameState::WaitingForBothPlayers;
      game->player1_tid = p1;
      game->player2_tid = p2;
      game->game_index  = game_index.value();

      // add pair to the lookup table
      player_to_game_ptr[game_index.value()][0] = p1;
      player_to_game_ptr[game_index.value()][1] = p2;

      // reset waitng
      waiting.reset();

      uart_printf(
          CONSOLE,
          "RPS server: Game started between player %d and player %d\r\n", p1,
          p2);

      // tell them they are both ready to play
      Message p1_ready{};
      p1_ready.type = MessageType::RPS_PLAY_READY;
      reply(p1, p1_ready);

      Message p2_ready{};
      p2_ready.type = MessageType::RPS_PLAY_READY;
      reply(p2, p2_ready);
    }
    break;
  }
  case MessageType::RPS_PLAY: {
    auto game_index = find_game_index_for_player(sender_tid);
    if (!game_index.has_value()) {
      reply_with_error(sender_tid);
      break;
    }
    auto game = &games[game_index.value()];

    bool is_player1   = game->player1_tid == sender_tid;
    int partner_index = is_player1 ? 1 : 0;
    int partner_tid =
        game->player1_tid == sender_tid ? game->player2_tid : game->player1_tid;

    if (game->game_state == Game::GameState::PartnerHasQuit) {
      game_index_allocator.free(game_index.value());
      player_to_game_ptr[game_index.value()][partner_index] = -1;
      uart_printf(CONSOLE,
                  "RPS server: Game ended between player %d and player %d\r\n",
                  sender_tid, partner_tid);
      Message msg{};
      msg.type = MessageType::RPS_PLAY_RESULT;
      msg.payload.rps_play_result.result =
          RPS::PlayResultMessage::Result::PLAYER_QUIT;
      reply(sender_tid, msg);
      break;
    }

    using Choice = RPS::PlayMessage::Choice;
    auto choice  = msg.payload.rps_play.choice;
    if (choice != Choice::ROCK && choice != Choice::PAPER &&
        choice != Choice::SCISSORS) {
      reply_with_error(sender_tid);
      break;
    }

    if (is_player1) {
      if (game->game_state == Game::GameState::WaitingForBothPlayers) {
        game->player1_choice = choice;
        game->game_state     = Game::GameState::WaitingForPlayer2;
        // don't reply as waiting for player 2
      } else if (game->game_state == Game::GameState::WaitingForPlayer1) {
        game->player1_choice = choice;
        game->game_state     = Game::GameState::Finished;
      } else {
        // playing when you cannot play results in an error.
        reply_with_error(sender_tid);
      }
    } else { // player 2
      if (game->game_state == Game::GameState::WaitingForBothPlayers) {
        game->player2_choice = choice;
        game->game_state     = Game::GameState::WaitingForPlayer1;
        // don't reply as waiting for player 1
      } else if (game->game_state == Game::GameState::WaitingForPlayer2) {
        game->player2_choice = choice;
        game->game_state     = Game::GameState::Finished;
      } else {
        // playing when you cannot play results in an error.
        reply_with_error(sender_tid);
      }
    }

    // evaluate game state
    using Result = RPS::PlayResultMessage::Result;
    if (game->game_state == Game::GameState::Finished) {
      // game is finished, we need to evaluate the result
      // check if tie, and if not default to p1 win
      auto p1_result = game->player1_choice == game->player2_choice
                           ? Result::TIE
                           : Result::WIN;
      auto p2_result = game->player1_choice == game->player2_choice
                           ? Result::TIE
                           : Result::LOSE;

      // check conditions where p2 wins instead
      if ((game->player1_choice == Choice::ROCK &&
           game->player2_choice == Choice::PAPER) ||
          (game->player1_choice == Choice::PAPER &&
           game->player2_choice == Choice::SCISSORS) ||
          (game->player1_choice == Choice::SCISSORS &&
           game->player2_choice == Choice::ROCK)) {
        p1_result = Result::LOSE;
        p2_result = Result::WIN;
      }

      Message p1_msg{};
      p1_msg.type                           = MessageType::RPS_PLAY_RESULT;
      p1_msg.payload.rps_play_result.result = p1_result;

      Message p2_msg{};
      p2_msg.type                           = MessageType::RPS_PLAY_RESULT;
      p2_msg.payload.rps_play_result.result = p2_result;

      // print game result
      uart_printf(CONSOLE,
                  "RPS server: Game result: player %d %s, player %d %s\r\n",
                  game->player1_tid, RPS::result_str(p1_result),
                  game->player2_tid, RPS::result_str(p2_result));

      // reset game state for next game
      game->game_state = Game::GameState::WaitingForBothPlayers;

      // reply to both players
      reply(game->player1_tid, p1_msg);
      reply(game->player2_tid, p2_msg);
    }
    break;
  }
  case MessageType::RPS_QUIT: {
    auto game_index = find_game_index_for_player(sender_tid);
    if (!game_index.has_value()) {
      reply_with_error(sender_tid);
      break;
    } else {
      auto game              = &games[game_index.value()];
      bool sender_is_player1 = game->player1_tid == sender_tid;
      auto partner_tid =
          sender_is_player1 ? game->player2_tid : game->player1_tid;

      // diff cases:
      bool quit_user        = false;
      bool deallocate_game  = false;
      bool reply_to_partner = false;
      switch (game->game_state) {
      case Game::GameState::WaitingForBothPlayers:
        // we only quit user because we don't know partner response yet.
        // partner could be play or quit at this point.
        quit_user = true;
        break;
      case Game::GameState::WaitingForPlayer1:
        if (sender_is_player1) {
          deallocate_game  = true;
          reply_to_partner = true;
        }
        break;
        // impossible for sender to be player 2 and to be waiting for player 1
        // as they should be blocked after playing.
      case Game::GameState::WaitingForPlayer2:
        if (!sender_is_player1) {
          deallocate_game  = true;
          reply_to_partner = true;
        }
        break;
        // impossible for sender to be player 1 and to be waiting for player 2
        // as they should be blocked after playing.
      case Game::GameState::PartnerHasQuit:
        // no need to reply to partner as they have already quit.
        deallocate_game = true;
        break;
      default:
        reply_with_error(sender_tid);
        break;
      }

      if (quit_user) {
        // deallocate corresponding player in player_to_game_ptr
        // this makes current game un-indexable using user that quit.
        player_to_game_ptr[game->game_index][sender_is_player1 ? 0 : 1] = -1;
        game->game_state = Game::GameState::PartnerHasQuit;
      }

      if (deallocate_game) {
        player_to_game_ptr[game->game_index][0] = -1;
        player_to_game_ptr[game->game_index][1] = -1;
        game_index_allocator.free(game->game_index);
        uart_printf(
            CONSOLE,
            "RPS server: Game ended between player %d and player %d\r\n",
            sender_tid, partner_tid);
      }

      Message reply_msg{};
      if (reply_to_partner) {
        reply_msg.type = MessageType::RPS_PLAYER_QUIT;
        reply(partner_tid, reply_msg);
      }
      reply_msg.type = MessageType::RPS_QUIT_ACK;
      reply(sender_tid, reply_msg);
    }
    break;
  }
  default:
    reply_with_error(sender_tid);
    break;
  }
};

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
  RPSServer server{};
  for (;;) {
    server.run();
  }
}
