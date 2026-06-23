#pragma once

#include "buffer.h"
#include "clock_server.h"
#include "debug.h"
#include "io_helpers.h"
#include "map.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include "time.h"
#include "train_server.h"
#include "train_state.h"
#include "uart_tx_server.h"
#include <cstddef>
#include <type_traits>

void train_tree_task();
void cal_speed_task();

template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  int waiting_ui_update_worker_tid = -1;
  int waiting_can_tx_worker_tid    = -1;
  bool simple_pacing_can_send      = true;

  struct TreeMailbox {
    Buffer<TC::TreeMsg, 16> msgs{};
    bool waiting = false;
  };

  Buffer<TC::TX, TX_BUFFER_SIZE> tx_buf;
  Map<int, TreeMailbox, 10> trees;

  struct CalibratingTrain {
    uint32_t num   = 0;
    uint32_t speed = 0;
  } calibrating_train{};

  bool debug_sensor = false;
  int cs_tid        = -1;
  int tx_tid        = -1;

  State state{};

  static void rx_can_worker();
  static void tx_can_worker();

  void maybe_tx() {
    if (!tx_buf.is_empty() && waiting_can_tx_worker_tid >= 0 &&
        simple_pacing_can_send) {
      reply(waiting_can_tx_worker_tid, tx_buf.pop().value());
      waiting_can_tx_worker_tid = -1;
      simple_pacing_can_send    = false;
    }
  }

  void publish_tree_update(const TC::TreeUpdate &update) {
    for (auto [tid, mailbox] : trees) {
      _assert(mailbox.msgs.push(TC::TreeMsg{update}), "TREE MAILBOX FULL");
      if (mailbox.waiting) {
        auto next_msg = mailbox.msgs.pop();
        reply(tid, next_msg.value());
        mailbox.waiting = false;
      }
    }
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
                .mrk = SpeedCmd(cmd.id, user_speed_to_mrk_level(cmd.value))});
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
            tx_buf.push(TC::TX{.mrk = DirectionCmd(cmd.id, !cmd.flag)});
            tx_buf.push(TC::TX{
                .mrk = SpeedCmd(cmd.id,
                                user_speed_to_mrk_level(loco.requested_speed)),
            });
          } else if constexpr (std::is_same_v<Command, UserCmd::Stop>) {
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_STOP)});
          } else if constexpr (std::is_same_v<Command, UserCmd::Go>) {
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});
          } else if constexpr (std::is_same_v<Command, UserCmd::Reset>) {
            State default_state{};

            tx_buf.push(
                TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});

            for (const TrainState &train : default_state.trains) {
              tx_buf.push(
                  TC::TX{.mrk = LightCmd(train.loco_id, train.light_on)});
              tx_buf.push(TC::TX{
                  .mrk = SpeedCmd(train.loco_id, user_speed_to_mrk_level(
                                                     train.requested_speed))});
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
          } else if constexpr (std::is_same_v<Command, UserCmd::RunTree>) {
            int tree_tid = create(4, train_tree_task);
            TreeMailbox mailbox{};
            mailbox.msgs.push(TC::TreeMsg{TC::InitTree{cmd.id, cmd.value}});
            trees.set(tree_tid, mailbox);
            _assert(tree_tid >= 0, "TREE TASK CREATE FAILED");
          } else if constexpr (std::is_same_v<Command, UserCmd::CalSpeed>) {
            calibrating_train.num   = cmd.id;
            calibrating_train.speed = cmd.value;
            int cal_tid             = create(4, cal_speed_task);
            _assert(cal_tid >= 0, "CAL SPEED TASK CREATE FAILED");
          } else if constexpr (std::is_same_v<Command, UserCmd::Quit>) {
            if (waiting_ui_update_worker_tid >= 0) {
              reply(waiting_ui_update_worker_tid, TC::Quit{});
              waiting_ui_update_worker_tid = -1;
            }
            if (waiting_can_tx_worker_tid >= 0) {
              reply(waiting_can_tx_worker_tid, TC::Quit{});
              waiting_can_tx_worker_tid = -1;
            }
          } else if constexpr (std::is_same_v<Command, UserCmd::DebugSensor>) {
            debug_sensor = cmd.enabled;
          } else {
            _assert(false, "UNHANDLED USER COMMAND");
          }
        },
        command);
  }

  void handle(const int tid, const TC::RX &msg) {
    auto mrk = decode_frame(msg.frame);

    // emit event for sensor B6 on trigger
    if (auto *sensor = std::get_if<SensorData>(&mrk)) {

      // if debug sensor, then debug puts the sensor
      if (debug_sensor && sensor->new_state != 0) {
        char bank        = 'A' + sensor->bank;
        uint32_t time_us = static_cast<uint32_t>(Time(cs_tid)) * TICK_TIME_US;
        Debug_Puts(tx_tid, "sensor trigger: ", bank, sensor->number, " at ",
                   format_time(time_us));
      }

      constexpr uint16_t SENSOR_B6_ID = ('B' - 'A') * 16 + 6;
      if (sensor->sensor_id == SENSOR_B6_ID && sensor->new_state != 0) {
        emit_event(Event::SENSOR_B6);
      }
    }

    state.update_from_mrk(mrk);
    simple_pacing_can_send = simple_pacing_can_send || (msg.frame.resp == 1);
    maybe_tx();
    publish_tree_update(TC::TreeUpdate{.mrk = mrk});
    if (state.is_dirty() && waiting_ui_update_worker_tid >= 0) {
      reply(waiting_ui_update_worker_tid, TC::UIUpdate{state});
      state.clear_dirty();
      waiting_ui_update_worker_tid = -1;
    }
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::UIReady &) {
    if (state.is_dirty()) {
      reply(tid, TC::UIUpdate{state});
      state.clear_dirty();
      return;
    }
    waiting_ui_update_worker_tid = tid;
  }

  void handle(const int tid, const TC::TXReady &) {
    waiting_can_tx_worker_tid = tid;
    maybe_tx();
  }

  void handle(const int tid, const TC::CLICmd &msg) {
    expand_user_command(msg.cmd);
    maybe_tx();
    reply(tid, TC::Ack{});
    if (std::get_if<UserCmd::Quit>(&msg.cmd)) {
      exit();
    }
  }

  void handle(const int tid, const TC::TreeReady &) {
    auto *mailbox = trees.get_ref(tid);
    if (mailbox == nullptr) {
      reply_with_error(tid);
      return;
    }

    auto next_msg = mailbox->msgs.pop();
    if (next_msg.has_value()) {
      reply(tid, next_msg.value());
      return;
    }

    mailbox->waiting = true;
  }

  void handle(const int tid, const TC::TreeExit &) {
    trees.remove(tid);
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::CalSpeedReady &) {
    reply(tid, TC::CalSpeedParams{.loco_id = calibrating_train.num,
                                  .speed   = calibrating_train.speed});
  }

  template <class T> void handle(int sender_tid, const T &) {
    reply_with_error(sender_tid);
  }

public:
  static constexpr auto TC_SERVER_NAME = "TCSERVER";
  TrainControlServer() {
    auto response = RegisterAs(TC_SERVER_NAME);
    _assert(response == 0, "TC_SERVER_NAME REGISTERAS FAILED");

    cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
    _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

    tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
    _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

    create(2, rx_can_worker);
    create(2, tx_can_worker);

    expand_user_command(UserCmd::RemoveTrains{});
    expand_user_command(UserCmd::Reset{});
  }

  void run() {
    int sender_tid;
    Message msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  }
};