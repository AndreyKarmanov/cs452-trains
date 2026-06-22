#pragma once

#include "buffer.h"
#include "debug.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include "time.h"
#include "train_state.h"
#include <cstddef>
#include <type_traits>

template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  int waiting_ui_update_worker_tid = -1;
  int waiting_can_tx_worker_tid    = -1;

  Buffer<TC::TX, TX_BUFFER_SIZE> tx_buf;

  State state{};
  static void tx_can_worker();

  void send_waiting_can_tx_worker_if_pending() {
    if (waiting_can_tx_worker_tid < 0 || tx_buf.is_empty()) {
      return;
    }

    reply(waiting_can_tx_worker_tid, tx_buf.pop().value());
    waiting_can_tx_worker_tid = -1;
  }

  void expand_user_command(const UserCmd::Cmd &command) {
    std::visit(
        [&](const auto &cmd) {
          // need to use decay_t to get the "raw" type, like LightCmd
          using Command = std::decay_t<decltype(cmd)>;

          if constexpr (std::is_same_v<Command, UserCmd::Light>) {
            tx_buf.push(TC::TX{.mrk = LightCmd(cmd.id, cmd.flag)});
          } else if constexpr (std::is_same_v<Command, UserCmd::Speed>) {
            tx_buf.push(TC::TX{
                .mrk = SpeedCmd(cmd.id, static_cast<uint16_t>(cmd.value))});
          } else if constexpr (std::is_same_v<Command, UserCmd::Switch>) {
            tx_buf.push(TC::TX{
                .mrk = SwitchCmd(static_cast<uint16_t>(cmd.id), cmd.flag)});
          } else if constexpr (std::is_same_v<Command, UserCmd::Reverse>) {
            auto loco = state.get_loco(cmd.id);
            if (loco.requested_speed == 0) {
              tx_buf.push(TC::TX{.mrk = DirectionCmd(cmd.id, !cmd.flag)});
              return;
            }
            tx_buf.push(TC::TX{.mrk = SpeedCmd(cmd.id, 0)});
            tx_buf.push(TC::TX{
                .mrk = DirectionCmd(cmd.id, !cmd.flag),
            });
            tx_buf.push(TC::TX{
                .mrk = SpeedCmd(cmd.id, loco.requested_speed),
            });
          } else if constexpr (std::is_same_v<Command, UserCmd::Stop>) {
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_STOP)});
          } else if constexpr (std::is_same_v<Command, UserCmd::Go>) {
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});
          } else if constexpr (std::is_same_v<Command, UserCmd::Reset>) {
            State default_state{};

            tx_buf.push(
                TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});

            for (const Train &train : default_state.trains) {
              tx_buf.push(
                  TC::TX{.mrk = LightCmd(train.loco_id, train.light_on)});
              tx_buf.push(TC::TX{
                  .mrk = SpeedCmd(train.loco_id, train.requested_speed)});
              tx_buf.push(
                  TC::TX{.mrk = DirectionCmd(train.loco_id, train.backward)});
            }

            for (uint32_t sw_id = 0; sw_id < 22; ++sw_id) {
              tx_buf.push(
                  TC::TX{.mrk = SwitchCmd(State::switch_id(sw_id),
                                          default_state.is_switch_straight(
                                              State::switch_id(sw_id)))});
            }

          } else if constexpr (std::is_same_v<Command, UserCmd::RemoveTrains>) {
            tx_buf.push(
                TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});
          }
        },
        command);
  }

  bool has_dirty_state() const {
    return state.sensors_dirty || state.switches_dirty || state.trains_dirty ||
           state.status_dirty;
  }

  void reply_waiting_ui_update_worker_if_dirty() {
    if (waiting_ui_update_worker_tid < 0 || !has_dirty_state()) {
      return;
    }

    reply(waiting_ui_update_worker_tid, TC::UIUpdate{state, 0});
    waiting_ui_update_worker_tid = -1;
    state.sensors_dirty          = false;
    state.switches_dirty         = false;
    state.trains_dirty           = false;
    state.status_dirty           = false;
  }

public:
  static constexpr auto TC_SERVER_NAME = "TCSERVER";
  TrainControlServer() {
    auto response = RegisterAs(TC_SERVER_NAME);
    _assert(response == 0, "TC_SERVER_NAME REGISTERAS FAILED");

    create(2, tx_can_worker);

    expand_user_command(UserCmd::RemoveTrains{});
    expand_user_command(UserCmd::Reset{});
  }

  void handle(const int tid, const TC::RX &msg) {
    state.update_from_mrk(msg.mrk);
    reply_waiting_ui_update_worker_if_dirty();
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::UIReady &) {
    if (has_dirty_state()) {
      reply(tid, TC::UIUpdate{state, 0});
      state.sensors_dirty  = false;
      state.switches_dirty = false;
      state.trains_dirty   = false;
      state.status_dirty   = false;
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
    expand_user_command(msg.cmd);
    send_waiting_can_tx_worker_if_pending();
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