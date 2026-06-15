#pragma once

#include <cstdint>
#define NS_MAX_NAME_LENGTH 32

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

  // io servers
  TX_INTERRUPT = 19,
  TX_SEND      = 20,
  TX_REPLY     = 21,

  RX_INTERRUPT       = 22,
  RX_INTERRUPT_REPLY = 23,
  RX_GETC            = 25,
  RX_GETC_REPLY      = 26,
};

namespace NS {
  struct RegisterMessage {
    char name[NS_MAX_NAME_LENGTH];
  };

  struct WhoIsMessage {
    char name[NS_MAX_NAME_LENGTH];
  };

  struct RegisterReplyMessage {
    enum class Status { SUCCESS = 0, FAILURE = -1, NAME_TOO_LONG = -2 } status;
  };

  struct WhoIsReplyMessage {
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
  struct TimeMessage {};

  struct TimeReplyMessage {
    uint32_t ticks;
  };

  struct DelayMessage {
    uint32_t ticks;
  };

  struct DelayUntilMessage {
    uint32_t ticks;
  };

  struct DelayReplyMessage {
    uint32_t ticks;
  };

  struct TickMessage {};

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

// todo: This will grow to be the size of the largest in union
// in future, when this has much more data, we want a smaller approach for hot
// message types
struct Message {
  MessageType type;

  union {
    int error_code;

    NS::RegisterMessage ns_register;
    NS::WhoIsMessage ns_who_is;
    NS::RegisterReplyMessage ns_register_reply;
    NS::WhoIsReplyMessage ns_who_is_reply;

    RPS::SetupMessage rps_setup;
    RPS::PlayMessage rps_play;
    RPS::QuitMessage rps_quit;
    RPS::PlayReadyMessage rps_play_ready;
    RPS::PlayResultMessage rps_play_result;
    RPS::QuitAckMessage rps_quit_ack;

    CS::TimeMessage cs_time;
    CS::TimeReplyMessage cs_time_reply;
    CS::DelayMessage cs_delay;
    CS::DelayUntilMessage cs_delay_until;
    CS::DelayReplyMessage cs_delay_reply;
    CS::TickMessage cs_tick;

    TX::SendMessage tx_send;

    RX::GetcMessage rx_getc;
    RX::GetcReplyMessage rx_getc_reply;

  } data;
};
