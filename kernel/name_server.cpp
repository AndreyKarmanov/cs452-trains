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
  int len = receive(&sender_tid, msg);

  if (len < static_cast<int>(sizeof(msg))) {
    reply_with_error(sender_tid);
    return;
  }

  switch (msg.type) {
  case MessageType::NAME_SERVER_REGISTER_AS: {
    Message reply_msg{};
    reply_msg.type = MessageType::NAME_SERVER_REGISTER_REPLY;
    reply_msg.payload.ns_register_reply.status =
        RegisterAs(msg.payload.ns_register.name, sender_tid);
    reply(sender_tid, reply_msg);
    break;
  }
  case MessageType::NAME_SERVER_WHO_IS: {
    auto tid = WhoIs(msg.payload.ns_who_is.name).value_or(-1);
    Message reply_msg{};
    reply_msg.type = MessageType::NAME_SERVER_WHO_IS_REPLY;
    reply_msg.payload.ns_who_is_reply.tid = tid;
    reply(sender_tid, reply_msg);
    break;
  }
  default:
    reply_with_error(sender_tid);
    break;
  }
}

void name_server_task() {
  NameServer server;
  for (;;) {
    server.run();
    yield();
  }
}

int RegisterAs(const char *name) {
  Message msg{};
  msg.type = MessageType::NAME_SERVER_REGISTER_AS;
  _assert(std::strlen(name) < MAX_NAME_LENGTH, "Name is too long");
  std::strncpy(msg.payload.ns_register.name, name, MAX_NAME_LENGTH);
  Message reply_msg{};
  int len = send(TID, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg)) ||
      reply_msg.type != MessageType::NAME_SERVER_REGISTER_REPLY) {
    return -1;
  }
  return reply_msg.payload.ns_register_reply.status;
}

int WhoIs(const char *name) {
  Message msg{};
  msg.type = MessageType::NAME_SERVER_WHO_IS;
  _assert(std::strlen(name) < MAX_NAME_LENGTH, "Name is too long");
  std::strncpy(msg.payload.ns_who_is.name, name, MAX_NAME_LENGTH);
  Message reply_msg{};
  int len = send(TID, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg)) ||
      reply_msg.type != MessageType::NAME_SERVER_WHO_IS_REPLY) {
    return -1;
  }
  return reply_msg.payload.ns_who_is_reply.tid;
}
