#pragma once

#include "buffer.h"
#include "clock_server.h"
#include "debug.h"
#include "map.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "static_string.h"
#include "syscall.h"
#include "time.h"
#include "train_server.h"
#include "train_state.h"
#include "uart_tx_server.h"
#include <cstddef>
#include <type_traits>

void train_tree_task();

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

  int cs_tid = -1;
  int tx_tid = -1;

  State state{};

  static void rx_can_worker();
  static void tx_can_worker();
  static void train_tick_worker();

  void maybe_tx() {
    if (!tx_buf.empty() && waiting_can_tx_worker_tid >= 0 &&
        simple_pacing_can_send) {
      reply(waiting_can_tx_worker_tid, tx_buf.pop().value());
      waiting_can_tx_worker_tid = -1;
      simple_pacing_can_send    = false;
    }
  }

  void spawn_tree_task(void (*entry)(), const TC::TreeMsg &init) {
    int tree_tid = create(4, entry);
    _assert(tree_tid >= 0, "TREE TASK CREATE FAILED");
    TreeMailbox mailbox{};
    mailbox.msgs.push(init);
    trees.set(tree_tid, mailbox);
  }

  void publish_tree_update(const TC::TreeUpdate &update) {
    for (auto [tid, mailbox] : trees) {
      _assert(mailbox.msgs.push(update), "TREE MAILBOX FULL");
      if (mailbox.waiting) {
        auto next_msg = mailbox.msgs.pop();
        reply(tid, next_msg.value());
        mailbox.waiting = false;
      }
    }
  }

  void expand_user_command(const TC::Cmd::Any &command) {
    std::visit(
        [&](const auto &cmd) {
          // need to use decay_t to get the "raw" type, like LightCmd
          using Command = std::decay_t<decltype(cmd)>;
          using namespace TC::Cmd;

          if constexpr (std::is_same_v<Command, Light>) {
            tx_buf.push(TC::TX{.mrk = LightCmd(cmd.id, cmd.on)});
          } else if constexpr (std::is_same_v<Command, Speed>) {
            tx_buf.push(TC::TX{
                .mrk = SpeedCmd(cmd.id, user_speed_to_mrk_level(cmd.value))});
          } else if constexpr (std::is_same_v<Command, Switch>) {
            tx_buf.push(TC::TX{
                .mrk = SwitchCmd(static_cast<uint16_t>(cmd.id), cmd.straight)});
          } else if constexpr (std::is_same_v<Command, Reverse>) {
            spawn_tree_task(reverse_tree_task, TC::InitTree{.loco_id = cmd.id,
                                                            .value   = 0,
                                                            .value2  = 0,
                                                            .state   = state});
          } else if constexpr (std::is_same_v<Command, Direction>) {
            tx_buf.push(TC::TX{.mrk = DirectionCmd(cmd.id, cmd.backward)});
          } else if constexpr (std::is_same_v<Command, Stop>) {
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_STOP)});
          } else if constexpr (std::is_same_v<Command, Go>) {
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});
          } else if constexpr (std::is_same_v<Command, Reset>) {
            State default_state{};

            tx_buf.push(
                TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});
            tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});

            for (const TrainState &train : default_state.trains) {
              tx_buf.push(
                  TC::TX{.mrk = LightCmd(train.loco_id, train.light_on)});
              tx_buf.push(TC::TX{
                  .mrk = SpeedCmd(train.loco_id,
                                  user_speed_to_mrk_level(train.req_speed))});
              tx_buf.push(
                  TC::TX{.mrk = DirectionCmd(train.loco_id, train.backward)});
            }

            for (uint32_t sw_id = 0; sw_id < 22; ++sw_id) {
              tx_buf.push(
                  TC::TX{.mrk = SwitchCmd(State::switch_id(sw_id),
                                          default_state.is_switch_straight(
                                              State::switch_id(sw_id)))});
            }
          } else if constexpr (std::is_same_v<Command, RemoveTrains>) {
            tx_buf.push(
                TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});
          } else if constexpr (std::is_same_v<Command, RunTree>) {
            spawn_tree_task(train_tree_task, TC::InitTree{.loco_id = cmd.id,
                                                          .value   = cmd.value,
                                                          .value2  = cmd.value2,
                                                          .state   = state});
          } else if constexpr (std::is_same_v<Command, Quit>) {
            if (waiting_ui_update_worker_tid >= 0) {
              reply(waiting_ui_update_worker_tid, TC::Quit{});
              waiting_ui_update_worker_tid = -1;
            }
            if (waiting_can_tx_worker_tid >= 0) {
              reply(waiting_can_tx_worker_tid, TC::Quit{});
              waiting_can_tx_worker_tid = -1;
            }
          } else if constexpr (std::is_same_v<Command, Nav>) {
            spawn_tree_task(nav_tree_task, TC::InitNav{.loco_id = cmd.id,
                                                       .to      = cmd.to,
                                                       .speed   = cmd.speed,
                                                       .offset  = cmd.offset,
                                                       .state   = state});
          } else {
            _assert(false, "UNHANDLED USER COMMAND");
          }
        },
        command);
  }

  void handle(const int tid, const TC::RX &msg) {
    auto mrk = decode_frame(msg.frame);
    state.update_from_mrk(mrk, msg.time);
    simple_pacing_can_send = simple_pacing_can_send || (msg.frame.resp == 1);
    maybe_tx();
    publish_tree_update(TC::TreeUpdate{.mrk = mrk, .time = msg.time});
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

  void handle(const int tid, const TC::Cmd::Any &msg) {
    expand_user_command(msg);
    maybe_tx();
    reply(tid, TC::Ack{});
    if (std::get_if<TC::Cmd::Quit>(&msg)) {
      exit();
    }
  }

  void handle(const int tid, const TC::TreeReady &msg) {
    auto *mailbox = trees.get_ref(tid);
    if (mailbox == nullptr) {
      reply_with_error(tid);
      return;
    }
    auto loco = state.get_loco(msg.train.loco_id);
    if (loco) {
      loco->ve_nm = msg.train.ve_nm;
      loco->v_max_umpt[msg.train.req_speed] =
          msg.train.v_max_umpt[msg.train.req_speed];
      loco->a_nmpt2[msg.train.req_speed] =
          msg.train.a_nmpt2[msg.train.req_speed];
      state.trains_dirty = true;
    }

    auto next_msg = mailbox->msgs.pop();
    if (next_msg.has_value()) {
      reply(tid, next_msg.value());
      return;
    }

    mailbox->waiting = true;
  }

  void handle(const int tid, const TC::TreeTick &msg) {
    reply(tid, TC::Ack{});
    publish_tree_update(TC::TreeUpdate{.mrk = UnknownCmd{}, .time = msg.time});
  }

  void handle(const int tid, const TC::TreeExit &) {
    trees.remove(tid);
    reply(tid, TC::Ack{});
  }

  template <class T> void handle(int sender_tid, const T &) {
    reply_with_error(sender_tid);
  }

public:
  static constexpr auto NAME                      = "TCSERVER";
  static constexpr auto TICKS_BETWEEN_TRAIN_TICKS = 10;
  TrainControlServer() {
    auto response = RegisterAs(NAME);
    _assert(response == 0, "TC  REGISTERAS FAILED");

    cs_tid = WhoIs(ClockServer<>::NAME);
    _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

    tx_tid = WhoIs(UART_TX_Server::NAME);
    _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

    create(1, rx_can_worker);
    create(2, tx_can_worker);
    create(5, train_tick_worker);

    expand_user_command(TC::Cmd::RemoveTrains{});
    expand_user_command(TC::Cmd::Reset{});
  }

  void run() {
    int sender_tid;
    Message msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  }
};