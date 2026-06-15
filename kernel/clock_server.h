#pragma once

#include "debug.h"
#include "heap.h"
#include "kernel_state.h"
#include "message.h"
#include "name_server.h"
#include "syscall.h"
#include <cstdint>
#include <stdint.h>
#include <utility>

template <size_t MAX_WAITING = MAX_TASKS> class ClockServer {
  uint32_t curr_tick{};
  Heap<std::pair<uint32_t, int>, MAX_WAITING> waiting_heap;

public:
  static constexpr auto CLOCK_SERVER_NAME = "CLOCKSERVER";

  ClockServer() {
    auto response = RegisterAs(CLOCK_SERVER_NAME);
    _assert(response == 0, "CLOCK SERVER REGISTERAS FAILED");
  }

  void handle(const int tid, const CS::Time &) {
    reply(tid, CS::TimeReply{.ticks = curr_tick});
  }

  void handle(const int tid, const CS::Delay &msg) {
    if (msg.ticks < 0) {
      reply_with_error_var(tid, -2);
      return;
    }

    waiting_heap.push({msg.ticks + curr_tick, tid});
  }

  void handle(const int tid, const CS::DelayUntil &msg) {
    if (msg.ticks < 0) {
      reply_with_error_var(tid, -2);
      return;
    }

    if (msg.ticks <= curr_tick) {
      reply(tid, CS::DelayReply{.ticks = curr_tick});
    } else {
      waiting_heap.push({msg.ticks, tid});
    }
  }

  void handle(const int tid, const CS::Tick &) {
    curr_tick++;
    reply(tid, CS::Tick{});

    while (true) {
      auto waiting_val_opt = waiting_heap.peek();
      if (!waiting_val_opt.has_value()) {
        break;
      }

      auto [wake_time, waiting_tid] = waiting_val_opt.value();
      if (wake_time > curr_tick) {
        break;
      }

      waiting_heap.pop();
      reply(waiting_tid, CS::DelayReply{.ticks = curr_tick});
    }
  }

  template <class T> void handle(const int tid, const T &) {
    reply_with_error_var(tid);
  }

  void run() {
    int sender_tid;
    MessageVar msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  }
};

void clock_server_task();

int Time(int tid);
int Delay(int tid, uint32_t ticks);
int DelayUntil(int tid, uint32_t ticks);

void test_clock_server();
void test_clock_client_task();