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

  struct Register {
    StaticString<MAX_NAME_LEN> name;
  };

  struct WhoIs {
    StaticString<MAX_NAME_LEN> name;
  };

  struct RegisterReply {
    enum class Status { SUCCESS = 0, FAILURE = -1, NAME_TOO_LONG = -2 } status;
  };

  struct WhoIsReply {
    int tid;
  };

} // namespace NS

namespace RPS {
  struct SetupMessage {};
  struct PlayMessage {
    enum class Choice { ROCK, PAPER, SCISSORS } choice;
  };

  inline const char *choice_str(PlayMessage::Choice choice) {
    switch (choice) {
    case PlayMessage::Choice::ROCK:
      return "rock";
    case PlayMessage::Choice::PAPER:
      return "paper";
    case PlayMessage::Choice::SCISSORS:
      return "scissors";
    }
    return "?";
  }

  struct QuitMessage {};

  struct PlayReadyMessage {};
  struct PlayResultMessage {
    enum class Result { WIN, LOSE, TIE, PLAYER_QUIT } result;
  };

  inline const char *result_str(PlayResultMessage::Result result) {
    switch (result) {
    case PlayResultMessage::Result::WIN:
      return "win";
    case PlayResultMessage::Result::LOSE:
      return "lose";
    case PlayResultMessage::Result::TIE:
      return "tie";
    case PlayResultMessage::Result::PLAYER_QUIT:
      return "partner quit";
    }
    return "?";
  }

  struct QuitAckMessage {};
} // namespace RPS

namespace CS {
  struct Time {};

  struct TimeReply {
    uint32_t ticks;
  };

  struct Delay {
    uint32_t ticks;
  };

  struct DelayUntil {
    uint32_t ticks;
  };

  struct DelayReply {
    uint32_t ticks;
  };

  struct Tick {};

} // namespace CS

namespace TX {
  constexpr int MAX_DATA_LENGTH = 256;

  struct SendMessage {
    int len;
    char data[MAX_DATA_LENGTH];
  };
} // namespace TX

namespace RX {
  struct GetcMessage {};

  struct GetcReplyMessage {
    char c;
  };
} // namespace RX

namespace FUT {
  struct ClientParamRequest {};
  struct ClientInitMessage {
    int delay_ticks;
    int delay_count;
  };

} // namespace FUT

struct ErrorMessage {
  int error_code;
};

// todo: This will grow to be the size of the largest in union
// in future, when this has much more data, we want a smaller approach for hot
// message types
struct Message {
  MessageType type;

  union {
    int error_code;

    NS::Register ns_register;
    NS::WhoIs ns_who_is;
    NS::RegisterReply ns_register_reply;
    NS::WhoIsReply ns_who_is_reply;

    RPS::SetupMessage rps_setup;
    RPS::PlayMessage rps_play;
    RPS::QuitMessage rps_quit;
    RPS::PlayReadyMessage rps_play_ready;
    RPS::PlayResultMessage rps_play_result;
    RPS::QuitAckMessage rps_quit_ack;

    CS::Time cs_time;
    CS::TimeReply cs_time_reply;
    CS::Delay cs_delay;
    CS::DelayUntil cs_delay_until;
    CS::DelayReply cs_delay_reply;
    CS::Tick cs_tick;

    TX::SendMessage tx_send;

    RX::GetcMessage rx_getc;
    RX::GetcReplyMessage rx_getc_reply;

    FUT::ClientParamRequest fut_params_req;
    FUT::ClientInitMessage fut_params;
  } data;
};

using MessageVar =
    std::variant<NS::Register, NS::WhoIs, NS::RegisterReply, NS::WhoIsReply,
                 RPS::SetupMessage, RPS::PlayMessage, RPS::QuitMessage,
                 RPS::PlayReadyMessage, RPS::PlayResultMessage,
                 RPS::QuitAckMessage, CS::Time, CS::TimeReply, CS::Delay,
                 CS::DelayUntil, CS::DelayReply, CS::Tick,
                 FUT::ClientParamRequest, FUT::ClientInitMessage, ErrorMessage>;

static_assert(std::is_trivially_copyable<MessageVar>::value,
              "MessageVar must be trivially copyable");