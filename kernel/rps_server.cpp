
#include <optional>

#include "debug.h"
#include "message.h"
#include "rps_server.h"
#include "syscall.h"

void RPSServer::run() {
  int sender_tid;
  Message msg{};
  int len = receive(&sender_tid, (char *)&msg, sizeof(msg));

  if (len < (int)sizeof(Message)) {
    reply_with_error(sender_tid);
    return;
  }

  switch (msg.type) {
  case MessageType::RPS_SIGNUP: {
    if (!waiting.has_value()) {
      // no one available, start to wait
      waiting.emplace(sender_tid);
    } else {
      auto p1 = waiting.value();
      auto p2 = sender_tid;

      _assert(p1 != p2, "STARTED GAME WITH SELF");

      // add pair to the map
      partners.set(p1, p2);
      partners.set(p2, p1);

      // reset waitng
      waiting.reset();

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
    if (!partners.contains(sender_tid)) {
      reply_with_error(sender_tid);
    } else {
      auto p1       = sender_tid;
      auto p2       = partners.get(sender_tid).value();
      auto p1_c     = msg.payload.rps_play.choice;
      auto p2_c_opt = choices.get(p2);

      // return if the other player hasn't played yet
      if (!p2_c_opt.has_value()) {
        choices.set(p1, p1_c); // save if p1 played first
        break;
      }
      auto p2_c = p2_c_opt.value();
      choices.remove(p2); // reset for next round

      using Choice = RPS::PlayMessage::Choice;
      using Result = RPS::PlayResultMessage::Result;
      // check if tie, and if not default to p1 win
      auto p1_result = p1_c == p2_c ? Result::TIE : Result::WIN;
      auto p2_result = p1_c == p2_c ? Result::TIE : Result::LOSE;

      // check conditions where p2 wins instead
      if ((p1_c == Choice::ROCK && p2_c == Choice::PAPER) ||
          (p1_c == Choice::PAPER && p2_c == Choice::SCISSORS) ||
          (p1_c == Choice::SCISSORS && p2_c == Choice::ROCK)) {
        p1_result = Result::LOSE;
        p2_result = Result::WIN;
      }

      Message p1_msg{};
      p1_msg.type                           = MessageType::RPS_PLAY_RESULT;
      p1_msg.payload.rps_play_result.result = p1_result;

      Message p2_msg{};
      p2_msg.type                           = MessageType::RPS_PLAY_RESULT;
      p2_msg.payload.rps_play_result.result = p2_result;

      reply(p1, p1_msg);
      reply(p2, p2_msg);
    };
    break;
  }
  case MessageType::RPS_QUIT: {
    if (!partners.contains(sender_tid)) {
      reply_with_error(sender_tid);
    } else {

      auto partner_tid = partners.get(sender_tid).value();

      // remove sender from games
      partners.remove(sender_tid);
      choices.remove(sender_tid);
      Message quit_ack{};
      quit_ack.type = MessageType::RPS_QUIT_ACK;
      reply(sender_tid, quit_ack);

      // remove partner from games
      // reply to them if they've already played
      partners.remove(partner_tid);
      if (choices.contains(partner_tid)) {
        choices.remove(partner_tid);
        Message player_quit{};
        player_quit.type = MessageType::RPS_PLAYER_QUIT;
        reply(partner_tid, player_quit);
      }
    }
    break;
  }
  default:
    reply_with_error(sender_tid);
    break;
  }
}

void rps_server_task() {
  RPSServer server{};
  while (true) {
    server.run();
  }
}
