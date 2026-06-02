#pragma once
#include "name_server.h"

enum class MessageType { NAME_SERVER_REGISTER_AS = 0, NAME_SERVER_WHO_IS = 1 };

struct NameServerRegisterMessage {
  char name[MAX_NAME_LENGTH];
};

struct NameServerWhoIsMessage {
  char name[MAX_NAME_LENGTH];
};

struct Message {
  MessageType type; // The Header: Always tells the receiver what this is

  union {
    NameServerRegisterMessage register_message;
    NameServerWhoIsMessage who_is_message;
  } payload; // The Payload
};