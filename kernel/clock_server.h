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
      if (msg.data.cs_delay.ticks < 0) {
        reply_with_error(tid, -2);
        break;
      }
      waiting_heap.push({msg.data.cs_delay.ticks + curr_tick, tid});
      break;
    }
    case MessageType::CS_DELAY_UNTIL: {
      if (msg.data.cs_delay_until.ticks < 0) {
        reply_with_error(tid, -2);
        break;
      }

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
      reply(tid, msg);
      while (true) {
        auto waiting_val_opt = waiting_heap.peek();
        if (!waiting_val_opt.has_value()) {
          break;
        }

        auto [wake_time, waiting_tid] = waiting_val_opt.value();
        if (wake_time > static_cast<int>(curr_tick)) {
          break;
        }

        waiting_heap.pop();
        Message reply_msg;
        reply_msg.type                      = MessageType::CS_DELAY_REPLY;
        reply_msg.data.cs_delay_reply.ticks = curr_tick;
        reply(waiting_tid, reply_msg);
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
