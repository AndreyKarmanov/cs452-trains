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
  static constexpr auto CLOCK_SERVER_NAME = "CLOCKSERVER";

public:
  ClockServer() {
    auto response = RegisterAs(CLOCK_SERVER_NAME);
    _assert(response == 0, "CLOCK SERVER REGISTERAS FAILED");
    create(2, clock_notifier_task);
  }

  static void clock_notifier_task() {
    int cs_tid = WhoIs(CLOCK_SERVER_NAME);
    _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

    Message msg;
    msg.type = MessageType::CS_TICK;
    Message rcv_msg;

    while (true) {
      await_event(Event::CLOCK_TICK_1MS);
      auto rcv_len = send(cs_tid, msg, rcv_msg);
      _assert(rcv_len >= 0, "CLOCK TICK FAILED");
    }
  }

  void run() {
    int tid;
    Message msg;
    auto rcv_size = receive(&tid, msg);

    switch (msg.type) {
    case MessageType::CS_TIME: {
      Message reply_msg;
      reply_msg.type                     = MessageType::CS_TIME_REPLY;
      reply_msg.data.cs_time_reply.ticks = curr_tick;
      reply(tid, reply_msg);
      break;
    }
    case MessageType::CS_DELAY: {
      waiting_heap.push({msg.data.cs_delay.ticks + curr_tick, tid});
      break;
    }
    case MessageType::CS_DELAY_UNTIL: {
      if (msg.data.cs_delay_until.ticks <= curr_tick) {
        Message reply_msg;
        reply_msg.type                      = MessageType::CS_DELAY_REPLY;
        reply_msg.data.cs_delay_reply.ticks = curr_tick;
        reply(tid, reply_msg);
      } else {
        waiting_heap.push({msg.data.cs_delay_until.ticks, tid});
      }
      break;
    }
    case MessageType::CS_TICK: {
      curr_tick++;
      auto waiting_val_opt = waiting_heap.peek();
      while (waiting_val_opt.has_value()) {
        auto [tid, wake_time] = waiting_val_opt.value();
        if (wake_time <= curr_tick) {
          waiting_heap.pop();
          Message reply_msg;
          reply_msg.type                      = MessageType::CS_DELAY_REPLY;
          reply_msg.data.cs_delay_reply.ticks = curr_tick;
          reply(tid, reply_msg);
        }
        waiting_val_opt = waiting_heap.peek();
      }
      break;
    }
    default:
      reply_with_error(tid);
      break;
    }
  }
};

void clock_server_task();

int Time(int tid);
int Delay(int tid, int ticks);
int DelayUntil(int tid, int ticks);
