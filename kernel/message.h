#pragma once

#include "static_string.h"
#include <cstdint>
#include <type_traits>
#include <variant>

enum class MessageType {
  // error is 0, so uninitalized is an error
  ERROR     = 0,
  TASK_EXIT = 1,

  // name server
  NS_REGISTER_AS    = 2,
  NS_WHO_IS         = 3,
  NS_REGISTER_REPLY = 4,
  NS_WHO_IS_REPLY   = 5,

  // to rps server
  RPS_SIGNUP = 6, // client asks to join a game
  RPS_PLAY   = 7, // client sends RPS choice
  RPS_QUIT   = 8, // client quits game

  // to rps client
  RPS_PLAY_READY  = 9,  // server notifies of game start
  RPS_PLAY_RESULT = 10, // server returns game result
  RPS_PLAYER_QUIT = 11, // server notifies other player quit
  RPS_QUIT_ACK    = 12, // server confirms player quit

  // clock server
  CS_TIME        = 13,
  CS_TIME_REPLY  = 14,
  CS_DELAY       = 15,
  CS_DELAY_UNTIL = 16,
  CS_DELAY_REPLY = 17,
  CS_TICK        = 18,

  FUT_CLIENT_PARAMS       = 19,
  FUT_CLIENT_PARAMS_REPLY = 20,

  // io servers
  TX_INTERRUPT = 21,
  TX_SEND      = 22,
  TX_REPLY     = 23,

  RX_INTERRUPT       = 24,
  RX_INTERRUPT_REPLY = 25,
  RX_GETC            = 27,
  RX_GETC_REPLY      = 28,
};

namespace NS {
  constexpr int NAMESERVER_TID  = 1;
  constexpr size_t MAX_NAME_LEN = 32;

  struct RegisterMsg {
    StaticString<MAX_NAME_LEN> name;
  };

  struct WhoIsMsg {
    StaticString<MAX_NAME_LEN> name;
  };

  struct RegisterReplyMsg {
    enum class Status { SUCCESS = 0, FAILURE = -1, NAME_TOO_LONG = -2 } status;
  };

  struct WhoIsReplyMsg {
    int tid;
  };

} // namespace NS

namespace RPS {
  struct SetupMsg {};
  struct PlayMsg {
    enum class Choice { ROCK, PAPER, SCISSORS } choice;
  };

  inline const char *choice_str(PlayMsg::Choice choice) {
    switch (choice) {
    case PlayMsg::Choice::ROCK:
      return "rock";
    case PlayMsg::Choice::PAPER:
      return "paper";
    case PlayMsg::Choice::SCISSORS:
      return "scissors";
    }
    return "?";
  }

  struct QuitMsg {};

  struct PlayReadyMsg {};
  struct PlayResultMsg {
    enum class Result { WIN, LOSE, TIE, PLAYER_QUIT } result;
  };

  inline const char *result_str(PlayResultMsg::Result result) {
    switch (result) {
    case PlayResultMsg::Result::WIN:
      return "win";
    case PlayResultMsg::Result::LOSE:
      return "lose";
    case PlayResultMsg::Result::TIE:
      return "tie";
    case PlayResultMsg::Result::PLAYER_QUIT:
      return "partner quit";
    }
    return "?";
  }

  struct QuitAckMsg {};
} // namespace RPS

namespace CS {
  struct TimeMsg {};

  struct TimeReplyMsg {
    uint32_t ticks;
  };

  struct DelayMsg {
    uint32_t ticks;
  };

  struct DelayUntilMsg {
    uint32_t ticks;
  };

  struct DelayReplyMsg {
    uint32_t ticks;
  };

  struct TickMsg {};

} // namespace CS

namespace TX {
  constexpr int MAX_DATA_LENGTH = 256;

  struct SendMsg {
    int len;
    char data[MAX_DATA_LENGTH];
  };

  struct InterruptMsg {};

  struct ReplyMsg {};
} // namespace TX

namespace RX {
  struct GetcMsg {};

  struct GetcReplyMsg {
    char c;
  };

  struct InterruptMsg {};

  struct InterruptReplyMsg {};
} // namespace RX

namespace FUT {
  struct ClientParamRequestMsg {};
  struct ClientInitMsg {
    int delay_ticks;
    int delay_count;
  };

} // namespace FUT

struct ErrorMsg {
  int error_code;
};

struct TaskExitMsg {};

using Message =
    std::variant<NS::RegisterMsg, NS::WhoIsMsg, NS::RegisterReplyMsg,
                 NS::WhoIsReplyMsg, RPS::SetupMsg, RPS::PlayMsg, RPS::QuitMsg,
                 RPS::PlayReadyMsg, RPS::PlayResultMsg, RPS::QuitAckMsg,
                 CS::TimeMsg, CS::TimeReplyMsg, CS::DelayMsg, CS::DelayUntilMsg,
                 CS::DelayReplyMsg, CS::TickMsg, FUT::ClientParamRequestMsg,
                 FUT::ClientInitMsg, TX::SendMsg, TX::InterruptMsg,
                 TX::ReplyMsg, RX::GetcMsg, RX::GetcReplyMsg, RX::InterruptMsg,
                 RX::InterruptReplyMsg, ErrorMsg, TaskExitMsg>;

static_assert(std::is_trivially_copyable<Message>::value,
              "MessageVar must be trivially copyable");