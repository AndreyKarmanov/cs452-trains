#pragma once

#include "buffer.h"
#include "debug.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include "train_state.h"
#include "uart.h"
#include <array>
#include <cstddef>

size_t expand_user_command(const State &state, const UserCmd &command,
                           std::array<MRKCmd, 64> &commands);

// todo: make the workers a class inside of StateServer
// todo: see if making the server subclasses & a real heirarchy work properly -
// perf perhaps? todo: add perf testing stuff more easily.
template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  Buffer<MRKCmd, TX_BUFFER_SIZE> tx_buf;
  int waiting_can_tx_worker_tid    = -1;
  int waiting_ui_update_worker_tid = -1;

  State state{};
  static void tx_can_worker();
  static void rx_can_worker();

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
    debug_printf(CONSOLE, "TCSERVER starting\n\r");
    auto response = RegisterAs(TC_SERVER_NAME);
    _assert(response == 0, "TC_SERVER_NAME REGISTERAS FAILED");

    create(4, tx_can_worker);
    create(5, rx_can_worker);
  }

  void handle(const int tid, const TC::RX &msg) {
    debug_printf(CONSOLE, "TCSERVER RX from %d\n\r", tid);
    state.update_from_mrk(msg.mrk);
    reply_waiting_ui_update_worker_if_dirty();
    debug_printf(CONSOLE, "RX update received\n\r");
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::UIReady &) {
    debug_printf(CONSOLE, "TCSERVER UIReady from %d\n\r", tid);
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
    debug_printf(CONSOLE, "TCSERVER TXReady from %d buffer=%d waiting=%d\n\r",
                 tid, tx_buf.is_empty() ? 0 : 1, waiting_can_tx_worker_tid);
    if (tx_buf.is_empty()) {
      waiting_can_tx_worker_tid = tid;
      return;
    }
    reply(tid, TC::TX{tx_buf.pop().value()});
  }

  void handle(const int tid, const TC::CLICmd &msg) {
    debug_printf(CONSOLE, "TCSERVER CLICmd from %d\n\r", tid);
    std::array<MRKCmd, 64> commands{};
    size_t command_count = expand_user_command(state, msg.cmd, commands);

    if (command_count == 0) {
      debug_printf(CONSOLE, "TCSERVER CLICmd expanded to 0 commands\n\r");
      reply(tid, TC::Ack{});
      return;
    }

    for (size_t i = 0; i < command_count; ++i) {
      debug_printf(CONSOLE, "TCSERVER enqueue command %u/%u\n\r",
                   static_cast<unsigned>(i + 1),
                   static_cast<unsigned>(command_count));
      if (waiting_can_tx_worker_tid >= 0) {
        debug_printf(CONSOLE, "TCSERVER replying to waiting TX worker %d\n\r",
                     waiting_can_tx_worker_tid);
        reply(waiting_can_tx_worker_tid, TC::TX{commands[i]});
        waiting_can_tx_worker_tid = -1;
        continue;
      }

      auto pushed = tx_buf.push(commands[i]);
      _assert(pushed, "TX BUFFER FULL");
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
    debug_printf(CONSOLE, "TCSERVER dispatch from %d\n\r", sender_tid);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  };
};