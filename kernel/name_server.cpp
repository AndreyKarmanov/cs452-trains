#include "name_server.h"
#include "debug.h"
#include "message.h"
#include "syscall.h"

// 0 for success, -1 for failure
int NameServer::RegisterAs(const char *name, int tid) {
  NameKey key{};
  std::strncpy(key.data, name, MAX_NAME_LENGTH);
  return name_to_tid.set(key, tid) ? 0 : -1;
};

std::optional<int> NameServer::WhoIs(const char *name) {
  NameKey key{};
  std::strncpy(key.data, name, MAX_NAME_LENGTH);
  return name_to_tid.get(key);
};

void NameServer::run() {
  int sender_tid;
  Message msg{};
  int len = receive(&sender_tid, (char *)&msg, sizeof(msg));
  // handle too long len here?

  switch (msg.type) {
  case MessageType::NAME_SERVER_REGISTER_AS: {
    int status = RegisterAs(msg.payload.register_message.name, sender_tid);
    reply(sender_tid, (const char *)&status, sizeof(status));
    break;
  }
  case MessageType::NAME_SERVER_WHO_IS: {
    auto tid   = WhoIs(msg.payload.who_is_message.name);
    int result = tid.has_value() ? tid.value() : -1;
    reply(sender_tid, (const char *)&result, sizeof(result));
    break;
  }
  default:
    break;
  }
}

int RegisterAs(const char *name) {
  Message msg{};
  msg.type = MessageType::NAME_SERVER_REGISTER_AS;
  std::strncpy(msg.payload.register_message.name, name, MAX_NAME_LENGTH);
  _assert(std::strlen(name) < MAX_NAME_LENGTH, "Name is too long");
  int status = -1;
  int len    = send(TID, (const char *)&msg, sizeof(msg), (char *)&status,
                    sizeof(status));
  if (len < (int)sizeof(status)) {
    return -1;
  }
  return status;
}

int WhoIs(const char *name) {
  Message msg{};
  msg.type = MessageType::NAME_SERVER_WHO_IS;
  std::strncpy(msg.payload.who_is_message.name, name, MAX_NAME_LENGTH);
  _assert(std::strlen(name) < MAX_NAME_LENGTH, "Name is too long");
  int tid = -1;
  int len =
      send(TID, (const char *)&msg, sizeof(msg), (char *)&tid, sizeof(tid));
  if (len < (int)sizeof(tid)) {
    return -1;
  }
  return tid;
}