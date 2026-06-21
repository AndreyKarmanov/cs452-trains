#pragma once

#include "buffer.h"
#include "debug.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include "train_state.h"
#include <array>
#include <cstddef>

size_t expand_user_command(const State &state, const UserCmd &command,
                           std::array<MRKCmd, 64> &commands);
void train_control_can_courier_task();

// todo: make the workers a class inside of StateServer
// todo: see if making the server subclasses & a real heirarchy work properly -
// perf perhaps? todo: add perf testing stuff more easily.
template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  int waiting_ui_update_worker_tid = -1;
  int waiting_can_tx_worker_tid    = -1;

  Buffer<TC::TX, TX_BUFFER_SIZE> tx_buf;

  State state{};
  static void rx_can_worker();

  void reply_waiting_can_tx_worker_if_pending() {
    if (waiting_can_tx_worker_tid < 0 || tx_buf.is_empty()) {
      return;
    }

    reply(waiting_can_tx_worker_tid, tx_buf.pop().value());
    waiting_can_tx_worker_tid = -1;
  }

  bool has_dirty_state() const {
    return state.sensors_dirty || state.switches_dirty || state.trains_dirty ||
           state.status_dirty || state.timings_dirty;
  }

  void reply_waiting_ui_update_worker_if_dirty() {
    if (waiting_ui_update_worker_tid < 0 || !has_dirty_state()) {
      return;
    }

    reply(waiting_ui_update_worker_tid, TC::UIUpdate{state});
    waiting_ui_update_worker_tid = -1;
    state.sensors_dirty          = false;
    state.switches_dirty         = false;
    state.trains_dirty           = false;
    state.status_dirty           = false;
    state.timings_dirty          = false;
  }

public:
  static constexpr auto TC_SERVER_NAME = "TCSERVER";
  TrainControlServer() {
    auto response = RegisterAs(TC_SERVER_NAME);
    _assert(response == 0, "TC_SERVER_NAME REGISTERAS FAILED");

    create(2, rx_can_worker);
  }

  void handle(const int tid, const TC::RX &msg) {
    state.update_from_mrk(msg.mrk);
    reply_waiting_ui_update_worker_if_dirty();
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::UIReady &) {
    if (has_dirty_state()) {
      reply(tid, TC::UIUpdate{state});
      state.sensors_dirty  = false;
      state.switches_dirty = false;
      state.trains_dirty   = false;
      state.status_dirty   = false;
      state.timings_dirty  = false;
      return;
    }

    waiting_ui_update_worker_tid = tid;
  }

  void handle(const int tid, const TC::TXReady &) {
    if (tx_buf.is_empty()) {
      waiting_can_tx_worker_tid = tid;
      return;
    }

    reply(tid, tx_buf.pop().value());
  }

  void handle(const int tid, const TC::CLICmd &msg) {
    std::array<MRKCmd, 64> commands{};
    size_t command_count = expand_user_command(state, msg.cmd, commands);

    if (command_count == 0) {
      reply(tid, TC::Ack{});
      return;
    }

    for (size_t i = 0; i < command_count; ++i) {
      auto pushed = tx_buf.push(TC::TX{.mrk = commands[i], .delay_ticks = 0});
      _assert(pushed, "TC TX BUFFER FULL");
      reply_waiting_can_tx_worker_if_pending();
    }

    reply(tid, TC::Ack{});
  }
  template <class T> void handle(int sender_tid, const T &) {
    reply_with_error(sender_tid);
  }

  void run() {
    int sender_tid;
    Message msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  };
};