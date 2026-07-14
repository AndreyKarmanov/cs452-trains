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

  struct TrackEdge {
    int node_idx;
    int dir;
  };

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

  namespace Tree {

    enum class Type {
      CALIBRATE,
      PRINT_TRAIN_STATS,
      STOP_MEASURE,
      REVERSE,
      NAVIGATE
    };

    struct Ready {
      TrainState train;
    };

    struct Tick {
      uint32_t time;
    };

    struct Exit {};

    struct Init {
      uint32_t loco_id;
      Type tree_type;
      int value1;
      int value2;
      int value3;

      State state;
    };

    struct TrackReserved {
      uint32_t loco_id;
      Buffer<TrackEdge, 16> path;
    };

    struct Update {
      MRKCmd mrk;
      uint32_t time;
    };

    using Msg = std::variant<Tree::Init, Tree::Update, Tree::TrackReserved>;

  } // namespace Tree

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
      Tree::Type tree_type;
      int value1;
      int value2;
      int value3;
    };

    struct Nav {
      uint32_t id;
      StaticString<8> to;
      uint32_t speed;
      int offset;
    };

    struct Reg {
      uint32_t id;
      StaticString<8> sensor;
    };

    struct Reserve {
      uint32_t id;
      Buffer<TrackEdge, 16> path;
    };

    struct ReleaseReserve {
      uint32_t id;
      Buffer<TrackEdge, 16> path;
    };

    using Any = std::variant<Invalid, Quit, Light, Speed, Switch, Reverse, Stop,
                             Go, Reset, RemoveTrains, RunTree, Direction, Nav,
                             Reg, Reserve, ReleaseReserve>;

  } // namespace Cmd

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
    FUT::ClientParamRequestMsg, FUT::ClientInitMsg, TX::SendMsg,
    TX::InterruptMsg, TX::ReplyMsg, RX::GetcMsg, RX::GetcReplyMsg,
    RX::InterruptMsg, RX::InterruptReplyMsg, ErrorMsg, TaskExitMsg, TC::UIReady,
    TC::UIUpdate, TC::Cmd::Any, TC::Quit, TC::TX, TC::RX, TC::TXReady,
    TC::Tree::Msg, TC::Tree::Ready, TC::Tree::Tick, TC::Tree::Exit, TC::Ack>;

static_assert(std::is_trivially_copyable<Message>::value,
              "MessageVar must be trivially copyable");