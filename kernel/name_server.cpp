#include "name_server.h"
#include "debug.h"
#include "message.h"
#include "syscall.h"

void name_server_task() {
  static NameServer server;
  for (;;) {
    server.run();
    yield();
  }
}

int RegisterAs(const char *name) {
  Message msg{};
  msg.type = MessageType::NS_REGISTER_AS;
  _assert(std::strlen(name) < NS_MAX_NAME_LENGTH, "Name is too long");
  std::strncpy(msg.data.ns_register.name, name, NS_MAX_NAME_LENGTH);
  Message reply_msg{};
  int len = send(NAMESERVER_TID, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg)) ||
      reply_msg.type != MessageType::NS_REGISTER_REPLY) {
    return -1;
  }
  return static_cast<int>(reply_msg.data.ns_register_reply.status);
}

int WhoIs(const char *name) {
  Message msg{};
  msg.type = MessageType::NS_WHO_IS;
  _assert(std::strlen(name) < NS_MAX_NAME_LENGTH, "Name is too long");
  std::strncpy(msg.data.ns_who_is.name, name, NS_MAX_NAME_LENGTH);
  Message reply_msg{};
  int len = send(NAMESERVER_TID, msg, reply_msg);
  if (len < static_cast<int>(sizeof(reply_msg)) ||
      reply_msg.type != MessageType::NS_WHO_IS_REPLY) {
    return -1;
  }
  return reply_msg.data.ns_who_is_reply.tid;
}