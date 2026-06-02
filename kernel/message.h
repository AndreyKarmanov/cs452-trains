#pragma once
#include "name_server.h"

enum class MessageType {
  // error is 0, so uninitalized is an error
  ERROR = 0,

  // name server
  NAME_SERVER_REGISTER_AS    = 2,
  NAME_SERVER_WHO_IS         = 3,
  NAME_SERVER_REGISTER_REPLY = 4,
  NAME_SERVER_WHO_IS_REPLY   = 5,

  // to rps server
  RPS_SIGNUP = 6, // client asks to join a game
  RPS_PLAY   = 7, // client sends RPS choice
  RPS_QUIT   = 8, // client quits game

  // to rps client
  RPS_PLAY_READY  = 9,  // server notifies of game start
  RPS_PLAY_RESULT = 10, // server returns game result
  RPS_PLAYER_QUIT = 11, // server notifies other player quit
  RPS_QUIT_ACK    = 12  // server confirms player quit
};

namespace NS {
  struct RegisterMessage {
    char name[MAX_NAME_LENGTH];
  };

  struct WhoIsMessage {
    char name[MAX_NAME_LENGTH];
  };

  struct RegisterReplyMessage {
    int status;
  };

  struct WhoIsReplyMessage {
    int status;
  };

} // namespace NS

namespace RPS {
  struct SetupMessage {};
  struct PlayMessage {
    enum class Choice { ROCK, PAPER, SCISSORS } choice;
  };
  struct QuitMessage {};

  struct PlayReadyMessage {
    int partner_tid;
  };
  struct PlayResultMessage {
    enum class Result { WIN, LOSE, TIE } result;
  };
  struct QuitAckMessage {};
} // namespace RPS

// todo: This will grow to be the size of the largest in union
// in future, when this has much more data, we want a smaller approach for hot
// message types
struct Message {
  MessageType type; // The Header: Always tells the receiver what this is

  union {
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

  } payload; // The Payload
};