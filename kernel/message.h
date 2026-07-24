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

  struct QuitMsg {};

  struct PlayReadyMsg {};
  struct PlayResultMsg {
    enum class Result { WIN, LOSE, TIE, PLAYER_QUIT } result;
  };

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
      CALIBRATE         = 0,
      PRINT_TRAIN_STATS = 1,
      STOP_MEASURE      = 2,
      REVERSE           = 3,
      NAVIGATE          = 4,
      FOREVER_NAVIGATE  = 5,
      LOOP_NODE         = 6
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

    struct Update {
      MRKCmd mrk;
      uint32_t time;
    };

    using Msg = std::variant<Tree::Init, Tree::Update>;

  } // namespace Tree

  namespace Cmd {
    struct Invalid {};
    struct Quit {};
    struct Light {
      uint32_t id;
      bool on;
    };
    struct Function {
      uint32_t id;
      uint8_t function;
      uint8_t value;
    };
    struct Speed {
      uint32_t id;
      uint32_t speed;
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
      int node_idx;
      uint32_t speed;
      int offset;
    };

    struct Reg {
      uint32_t id;
      int node_idx;
    };

    struct Reserve {
      uint32_t id;
      int node_idx;
      int edge_dir;
    };

    struct ReleaseReserve {
      uint32_t id;
      int node_idx;
      int edge_dir;
    };

    using Any = std::variant<Invalid, Quit, Light, Function, Speed, Switch,
                             Reverse, Stop, Go, Reset, RemoveTrains, RunTree,
                             Direction, Nav, Reg, Reserve, ReleaseReserve>;

    static_assert(std::is_trivially_copyable<Any>::value,
                  "TC::Cmd::Any must be trivially copyable");

  } // namespace Cmd

  struct Ack {
    int return_code{0};
  };
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