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

struct NameServerRegisterMessage {
  char name[MAX_NAME_LENGTH];
};

struct NameServerWhoIsMessage {
  char name[MAX_NAME_LENGTH];
};

struct NameServerRegisterReplyMessage {
  int status;
};

struct NameServerWhoIsReplyMessage {
  int status;
};

struct RPSSetupMessage {};
struct RPSPlayMessage {
  enum class Choice { Rock, Paper, Scissors } choice;
};
struct RPSQuitMessage {};

struct RPSPlayReadyMessage {
  int partner_tid;
};
struct RPSPlayResultMessage {
  int winner_tid;
};
struct RPSPlayerQuitMessage {};
struct RPSQuitAckMessage {};

// todo: This will grow to be the size of the largest in union
// in future, when this has much more data, we want a smaller approach for hot
// message types
struct Message {
  MessageType type; // The Header: Always tells the receiver what this is

  union {
    NameServerRegisterMessage ns_register;
    NameServerWhoIsMessage ns_who_is;
    NameServerRegisterReplyMessage ns_register_reply;
    NameServerWhoIsReplyMessage ns_who_is_reply;

    RPSSetupMessage rps_setup;
    RPSPlayMessage rps_play;
    RPSQuitMessage rps_quit;
    RPSPlayReadyMessage rps_play_ready;
    RPSPlayResultMessage rps_play_result;
    RPSPlayerQuitMessage rps_player_quit;
    RPSQuitAckMessage rps_quit_ack;

  } payload; // The Payload
};