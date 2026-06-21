#pragma once

#include "buffer.h"
#include "debug.h"
#include "heap.h"
#include "kernel_state.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include <cstdint>

template <size_t MAX_DELAYED = MAX_TASKS, size_t MAX_IMMEDIATE = MAX_TASKS>
class CanServer {
  struct PendingFrame {
    uint32_t wake_tick;
    CANFRAME frame;
  };

  struct PendingFrameCompare {
    bool operator()(const PendingFrame &lhs, const PendingFrame &rhs) const {
      return lhs.wake_tick < rhs.wake_tick;
    }
  };

  uint32_t curr_tick{};
  Buffer<CANFRAME, MAX_IMMEDIATE> immediate_queue;
  Heap<PendingFrame, MAX_DELAYED, PendingFrameCompare> delayed_queue;
  int waiting_tx_worker_tid = -1;

  static void tx_can_worker();
  static void tick_can_worker();

  void promote_due_frames() {
    while (true) {
      auto pending = delayed_queue.peek();
      if (!pending.has_value()) {
        break;
      }

      auto pending_frame = pending.value();
      if (pending_frame.wake_tick > curr_tick) {
        break;
      }

      delayed_queue.pop();
      auto pushed = immediate_queue.push(pending_frame.frame);
      _assert(pushed, "CAN IMMEDIATE QUEUE FULL");
    }
  }

  void reply_waiting_worker_if_possible() {
    if (waiting_tx_worker_tid < 0 || immediate_queue.is_empty()) {
      return;
    }

    reply(waiting_tx_worker_tid,
          CAN::TXMsg{.frame = immediate_queue.pop().value()});
    waiting_tx_worker_tid = -1;
  }

public:
  static constexpr auto CAN_SERVER_NAME = "CANSERVER";

  CanServer() {
    auto response = RegisterAs(CAN_SERVER_NAME);
    _assert(response == 0, "CAN SERVER REGISTERAS FAILED");

    create(2, tx_can_worker);
    create(2, tick_can_worker);
  }

  void handle(int tid, const CAN::SendMsg &msg) {
    auto pushed = immediate_queue.push(msg.frame);
    _assert(pushed, "CAN IMMEDIATE QUEUE FULL");
    reply_waiting_worker_if_possible();
    reply(tid, CAN::AckMsg{});
  }

  void handle(int tid, const CAN::DelayMsg &msg) {
    auto pushed = delayed_queue.push(PendingFrame{
        .wake_tick = curr_tick + msg.delay_ticks, .frame = msg.frame});
    _assert(pushed, "CAN DELAY QUEUE FULL");
    reply(tid, CAN::AckMsg{});
  }

  void handle(int tid, const CAN::DelayUntilMsg &msg) {
    if (msg.ticks <= curr_tick) {
      auto pushed = immediate_queue.push(msg.frame);
      _assert(pushed, "CAN IMMEDIATE QUEUE FULL");
      reply_waiting_worker_if_possible();
      reply(tid, CAN::AckMsg{});
      return;
    }

    auto pushed = delayed_queue.push(
        PendingFrame{.wake_tick = msg.ticks, .frame = msg.frame});
    _assert(pushed, "CAN DELAY QUEUE FULL");
    reply(tid, CAN::AckMsg{});
  }

  void handle(int tid, const CAN::TXReadyMsg &) {
    if (immediate_queue.is_empty()) {
      waiting_tx_worker_tid = tid;
      return;
    }

    reply(tid, CAN::TXMsg{.frame = immediate_queue.pop().value()});
  }

  void handle(int tid, const CAN::TickMsg &) {
    curr_tick += 10;
    promote_due_frames();
    reply_waiting_worker_if_possible();
    reply(tid, CAN::AckMsg{});
  }

  template <class T> void handle(int tid, const T &) { reply_with_error(tid); }

  void run() {
    int sender_tid;
    Message msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  }
};

void can_server_task();

int CanSend(int tid, const CANFRAME &frame);
int CanDelay(int tid, const CANFRAME &frame, uint32_t delay_ticks);
int CanDelayUntil(int tid, const CANFRAME &frame, uint32_t ticks);
