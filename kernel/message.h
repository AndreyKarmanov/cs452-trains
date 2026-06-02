#pragma once
#include "name_server.h"

typedef enum {
  NAME_SERVER_REGISTER_AS = 0,
  NAME_SERVER_WHO_IS      = 1
} MessageType;

typedef struct {
  char name[MAX_NAME_LENGTH];
} NameServerRegisterMessage;

typedef struct {
  char name[MAX_NAME_LENGTH];
} NameServerWhoIsMessage;

typedef struct {
  MessageType type; // The Header: Always tells the receiver what this is

  union {
    NameServerRegisterMessage register_message;
    NameServerWhoIsMessage who_is_message;
  } payload; // The Payload
} Message;