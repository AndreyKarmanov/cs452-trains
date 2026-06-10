#pragma once

#include <cstring>

#include "buffer.h"
#include "map.h"
#include "message.h"
#include "static_string.h"
#include "syscall.h"

constexpr static int NAMESERVER_TID = 1;

template <size_t MAX_NAMES = 32, size_t _MAX_NAME_LENGTH = NS_MAX_NAME_LENGTH>
class NameServer {
private:
  using name_t = StaticString<_MAX_NAME_LENGTH>;
  Map<name_t, int, MAX_NAMES> name_to_tid;
  Map<name_t, Buffer<int, MAX_NAMES>, MAX_NAMES> waiting_who_is;

public:
  void run() {
    int sender_tid;
    Message msg{};
    int len = receive(&sender_tid, msg);

    if (len < static_cast<int>(sizeof(msg))) {
      reply_with_error(sender_tid);
      return;
    }

    switch (msg.type) {
    case MessageType::NS_REGISTER_AS: {
      Message reply_msg{};
      reply_msg.type = MessageType::NS_REGISTER_REPLY;
      reply_msg.data.ns_register_reply.status =
          NS::RegisterReplyMessage::Status::SUCCESS;
      name_to_tid.set(msg.data.ns_register.name, sender_tid);
      reply(sender_tid, reply_msg);

      if (auto *buffer = waiting_who_is.get_ref(msg.data.ns_register.name)) {
        auto waiting_tid_opt = buffer->pop();

        Message who_is_reply_msg{};
        who_is_reply_msg.type = MessageType::NS_WHO_IS_REPLY;
        who_is_reply_msg.data.ns_who_is_reply.tid = sender_tid;
        while (waiting_tid_opt.has_value()) {
          int waiting_tid = waiting_tid_opt.value();
          reply(waiting_tid, who_is_reply_msg);
          waiting_tid_opt = buffer->pop();
        }
        waiting_who_is.remove(msg.data.ns_register.name);
      }
      break;
    }
    case MessageType::NS_WHO_IS: {
      auto tid_opt = name_to_tid.get(msg.data.ns_who_is.name);
      if (tid_opt.has_value()) {
        Message reply_msg{};
        reply_msg.type = MessageType::NS_WHO_IS_REPLY;
        reply_msg.data.ns_who_is_reply.tid = tid_opt.value();
        reply(sender_tid, reply_msg);
      } else {
        if (auto *buffer = waiting_who_is.get_ref(msg.data.ns_who_is.name)) {
          buffer->push(sender_tid);
        } else {
          Buffer<int, MAX_NAMES> waiting_buffer{};
          waiting_buffer.push(sender_tid);
          waiting_who_is.set(msg.data.ns_who_is.name, waiting_buffer);
        }
      }
      break;
    }
    default:
      reply_with_error(sender_tid);
      break;
    }
  };
};

void name_server_task();
int RegisterAs(const char *name);
int WhoIs(const char *name);