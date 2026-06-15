#pragma once

#include "buffer.h"
#include "debug.h"
#include "map.h"
#include "message.h"
#include "static_string.h"
#include "syscall.h"
#include <cstring>

constexpr static int NAMESERVER_TID = 1;

template <size_t MAX_NAMES = 32, size_t _MAX_NAME_LENGTH = NS::MAX_NAME_LEN>
class NameServer {
private:
  using name_t = StaticString<_MAX_NAME_LENGTH>;
  Map<name_t, int, MAX_NAMES> name_to_tid;
  Map<name_t, Buffer<int, MAX_NAMES>, MAX_NAMES> waiting_who_is;

public:
  void handle(const int sender_tid, const NS::Register &m) {
    NS::RegisterReply reply_msg{.status = NS::RegisterReply::Status::SUCCESS};
    name_to_tid.set(m.name, sender_tid);
    reply(sender_tid, reply_msg);

    if (auto *buffer = waiting_who_is.get_ref(m.name)) {
      auto waiting_tid_opt = buffer->pop();
      NS::WhoIsReply who_is_reply_msg{.tid = sender_tid};
      while (waiting_tid_opt.has_value()) {
        int waiting_tid = waiting_tid_opt.value();
        reply(waiting_tid, who_is_reply_msg);
        waiting_tid_opt = buffer->pop();
      }
      waiting_who_is.remove(m.name);
    }
  };

  void handle(const int sender_tid, const NS::WhoIs &m) {
    auto tid_opt = name_to_tid.get(m.name);
    if (tid_opt.has_value()) {
      NS::WhoIsReply reply_msg{.tid = tid_opt.value()};
      reply(sender_tid, reply_msg);
    } else {
      if (auto *buffer = waiting_who_is.get_ref(m.name)) {
        buffer->push(sender_tid);
      } else {
        Buffer<int, MAX_NAMES> waiting_buffer{};
        waiting_buffer.push(sender_tid);
        waiting_who_is.set(m.name, waiting_buffer);
      }
    }
  };

  template <class T> void handle(int sender_tid, const T &) {
    reply_with_error_var(sender_tid);
  }

  void run() {
    int sender_tid;
    MessageVar msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  };
};

void name_server_task();
int RegisterAs(const StaticString<NS::MAX_NAME_LEN> &name);
int WhoIs(const StaticString<NS::MAX_NAME_LEN> &name);

void test_name_server();
