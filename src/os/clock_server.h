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

  static void clock_tick_task();

public:
  static constexpr auto NAME = "CLOCKSERVER";

  // perhaps make hte clock server self-sufficient? run the notifier from this.
  ClockServer() {
    auto response = RegisterAs(NAME);
    _assert(response == 0, "CLOCK SERVER REGISTERAS FAILED");

    create(0, clock_tick_task);
  }

  void handle(const int tid, const CS::TimeMsg &) {
    reply(tid, CS::TimeReplyMsg{.ticks = curr_tick});
  }

  void handle(const int tid, const CS::DelayMsg &msg) {
    if (msg.ticks < 0) {
      reply_with_error(tid, -2);
      return;
    }

    waiting_heap.push({msg.ticks + curr_tick, tid});
  }

  void handle(const int tid, const CS::DelayUntilMsg &msg) {
    if (msg.ticks < 0) {
      reply_with_error(tid, -2);
      return;
    }

    if (msg.ticks <= curr_tick) {
      reply(tid, CS::DelayReplyMsg{.ticks = curr_tick});
    } else {
      waiting_heap.push({msg.ticks, tid});
    }
  }

  void handle(const int tid, const CS::TickMsg &) {
    curr_tick++;
    reply(tid, CS::TickMsg{});

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
      reply(waiting_tid, CS::DelayReplyMsg{.ticks = curr_tick});
    }
  }

  template <class T> void handle(const int tid, const T &) {
    reply_with_error(tid);
  }

  void run() {
    int sender_tid;
    Message msg;
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