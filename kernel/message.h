#pragma once

#include "mrk.h"
#include "static_string.h"
#include "train_state.h"
#include <cstdint>
#include <type_traits>
#include <variant>

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

namespace CAN {
  struct SendMsg {
    CANFRAME frame;
  };

  struct DelayMsg {
    CANFRAME frame;
    uint32_t delay_ticks;
  };

  struct DelayUntilMsg {
    CANFRAME frame;
    uint32_t ticks;
  };

  struct TXReadyMsg {};

  struct TXMsg {
    CANFRAME frame;
  };

  struct TickMsg {
    uint32_t ticks;
  };

  struct AckMsg {};
} // namespace CAN

namespace TX {
  constexpr int MAX_DATA_LENGTH = 512;

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

namespace TC {
  struct UIReady {};
  struct UIUpdate {
    State state;
    uint32_t time;
  };

  struct TXReady {};
  struct TX {
    MRKCmd mrk;
    uint32_t delay_ticks = 0;
  };

  struct RX {
    MRKCmd mrk;
  };

  struct CLIInput {
    char c;
    uint32_t time;
  };

  struct CLICmdReady {};

  struct CLICmd {
    UserCmd cmd;
  };

  struct Ack {};
  struct Quit {};
} // namespace TC

struct ErrorMsg {
  int error_code;
};

struct TaskExitMsg {};

using Message = std::variant<
    NS::RegisterMsg, NS::WhoIsMsg, NS::RegisterReplyMsg, NS::WhoIsReplyMsg,
    RPS::SetupMsg, RPS::PlayMsg, RPS::QuitMsg, RPS::PlayReadyMsg,
    RPS::PlayResultMsg, RPS::QuitAckMsg, CS::TimeMsg, CS::TimeReplyMsg,
    CS::DelayMsg, CS::DelayUntilMsg, CS::DelayReplyMsg, CS::TickMsg,
    CAN::SendMsg, CAN::DelayMsg, CAN::DelayUntilMsg, CAN::TXReadyMsg,
    CAN::TXMsg, CAN::TickMsg, CAN::AckMsg, FUT::ClientParamRequestMsg,
    FUT::ClientInitMsg, TX::SendMsg, TX::InterruptMsg, TX::ReplyMsg,
    RX::GetcMsg, RX::GetcReplyMsg, RX::InterruptMsg, RX::InterruptReplyMsg,
    ErrorMsg, TaskExitMsg, TC::UIReady, TC::UIUpdate, TC::CLICmdReady,
    TC::CLICmd, TC::CLIInput, TC::TX, TC::RX, TC::TXReady, TC::Ack>;

static_assert(std::is_trivially_copyable<Message>::value,
              "MessageVar must be trivially copyable");