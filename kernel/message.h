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
  };

  struct TXReady {};
  struct TX {
    MRKCmd mrk;
  };

  struct RX {
    CANFRAME frame;
    uint32_t time;
  };

  namespace Cmd {
    struct Invalid {};
    struct Quit {};
    struct Light {
      uint32_t id;
      bool on;
    };
    struct Speed {
      uint32_t id;
      uint32_t value;
    };
    struct Switch {
      uint32_t id;
      bool straight;
    };
    struct Reverse {
      uint32_t id;
      bool flag;
    };
    struct Direction {
      uint32_t id;
      bool backward;
    };
    struct Stop {};
    struct Go {};
    struct Reset {};
    struct RemoveTrains {};
    struct RunTree {
      uint32_t id;
      uint32_t value;
    };

    struct CalSpeed {
      uint32_t id;
      uint32_t value;
    };

    struct DebugSensor {
      bool enabled;
    };

    struct Nav {
      uint32_t id;
      StaticString<8> to;
      uint32_t speed;
    };

    using Any = std::variant<Invalid, Quit, Light, Speed, Switch, Reverse, Stop,
                             Go, Reset, RemoveTrains, RunTree, CalSpeed,
                             DebugSensor, Direction, Nav>;

  } // namespace Cmd

  struct TreeReady {
    TrainState train;
  };
  struct TreeExit {};

  struct InitTree {
    uint32_t loco_id;
    uint32_t value;
    State state;
  };

  struct InitNav {
    uint32_t loco_id;
    StaticString<8> to;
    uint32_t speed;
    State state;
  };

  struct TreeUpdate {
    MRKCmd mrk;
    uint32_t time;
  };

  using TreeMsg = std::variant<InitTree, InitNav, TreeUpdate>;

  struct CalSpeedReady {};

  struct CalSpeedParams {
    uint32_t loco_id;
    uint32_t speed;
  };

  struct TreeTick {
    uint32_t time;
  };
  struct Ack {};
  struct Quit {};
} // namespace TC

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
                 RX::InterruptReplyMsg, ErrorMsg, TaskExitMsg, TC::UIReady,
                 TC::UIUpdate, TC::Cmd::Any, TC::Quit, TC::TX, TC::RX,
                 TC::TXReady, TC::TreeReady, TC::TreeExit, TC::TreeMsg,
                 TC::TreeTick, TC::Ack, TC::CalSpeedReady, TC::CalSpeedParams>;

static_assert(std::is_trivially_copyable<Message>::value,
              "MessageVar must be trivially copyable");