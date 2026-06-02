#include "rps_server.h"
#include "message.h"
#include "syscall.h"

void RPSServer::run() {
  int sender_tid;
  Message msg{};
  int len = receive(&sender_tid, (char *)&msg, sizeof(msg));

  if (len < (int)sizeof(Message)) {
    reply_with_error(sender_tid);
    return;
  }

  switch (msg.type) {
  case MessageType::RPS_SIGNUP: {
    if (!waiting_tid.has_value()) {
      waiting_tid.emplace(sender_tid);
    } else if (waiting_tid.value() == sender_tid) {
      reply_with_error(waiting_tid.value());
    } else {
      auto p1 = waiting_tid.value();
      auto p2 = sender_tid;

      // add both to the map
      games.set(p1, p2);
      games.set(p2, p1);
      reply(p1, {.type = MessageType::RPS_PLAY_READY});
    }
    break;
  }
  case MessageType::RPS_PLAY: {
    break;
  }
  case MessageType::RPS_QUIT: {
    break;
  }
  default:
    reply_with_error(sender_tid);
    break;
  }
}

void rps_server_task() {
  RPSServer server{};
  while (true) {
    server.run();
  }
}
