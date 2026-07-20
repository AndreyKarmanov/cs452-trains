#pragma once

#include "buffer.h"
#include "clock_server.h"
#include "debug.h"
#include "io_helpers.h"
#include "map.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "overloaded.h"
#include "pathfind.h"
#include "static_string.h"
#include "syscall.h"
#include "time.h"
#include "track_node.h"
#include "train_state.h"
#include "train_trees.h"
#include "uart_tx_server.h"
#include <cstddef>

template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  int waiting_ui_update_worker_tid = -1;
  int waiting_can_tx_worker_tid    = -1;
  bool simple_pacing_can_send      = true;

  Track track;

  struct TreeMailbox {
    Buffer<TC::Tree::Msg, 16> msgs{};
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

  void spawn_tree_task(void (*entry)(), const TC::Tree::Msg &init) {
    int tree_tid = create(4, entry);
    _assert(tree_tid >= 0, "TREE TASK CREATE FAILED");
    TreeMailbox mailbox{};
    mailbox.msgs.push(init);
    trees.set(tree_tid, mailbox);
  }

  void publish_tree_update(const TC::Tree::Msg &update) {
    for (const auto &[tid, mailbox] : trees) {
      _assert(mailbox.msgs.push(update), "TREE MAILBOX FULL");
      if (mailbox.waiting) {
        auto next_msg = mailbox.msgs.pop();
        reply(tid, next_msg.value());
        mailbox.waiting = false;
      }
    }
  }

  bool expand_user_command(const TC::Cmd::Any &command) {
    return std::visit(
        Overloaded{
            [&](const TC::Cmd::Light &cmd) {
              tx_buf.push(TC::TX{.mrk = LightCmd(cmd.id, cmd.on)});
              return true;
            },
            [&](const TC::Cmd::Function &cmd) {
              tx_buf.push(
                  TC::TX{.mrk = FunctionCmd(cmd.id, cmd.function, cmd.value)});
              return true;
            },
            [&](const TC::Cmd::Speed &cmd) {
              tx_buf.push(TC::TX{
                  .mrk = SpeedCmd(cmd.id, user_speed_to_mrk_level(cmd.value))});
              return true;
            },
            [&](const TC::Cmd::Switch &cmd) {
              tx_buf.push(TC::TX{.mrk = SwitchCmd(static_cast<uint16_t>(cmd.id),
                                                  cmd.straight)});
              return true;
            },
            [&](const TC::Cmd::Reverse &cmd) {
              spawn_tree_task(run_tree,
                              TC::Tree::Init{
                                  .loco_id   = cmd.id,
                                  .tree_type = TC::Tree::Type::REVERSE,
                                  .value1    = 0,
                                  .value2    = 0,
                                  .value3    = 0,
                                  .state     = state,
                              });
              return true;
            },
            [&](const TC::Cmd::Direction &cmd) {
              tx_buf.push(TC::TX{.mrk = DirectionCmd(cmd.id, cmd.backward)});
              return true;
            },
            [&](const TC::Cmd::Stop &) {
              tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_STOP)});
              return true;
            },
            [&](const TC::Cmd::Go &) {
              tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});
              return true;
            },
            [&](const TC::Cmd::Reset &) {
              State default_state{};

              tx_buf.push(
                  TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});
              tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});

              for (const TrainState &train : default_state.trains) {
                tx_buf.push(TC::TX{.mrk = LightCmd(train.id, train.light_on)});
                tx_buf.push(TC::TX{
                    .mrk = SpeedCmd(train.id,
                                    user_speed_to_mrk_level(train.req_speed))});
                tx_buf.push(
                    TC::TX{.mrk = DirectionCmd(train.id, train.backward)});
              }

              for (uint32_t sw_id = 0; sw_id < 22; ++sw_id) {
                tx_buf.push(
                    TC::TX{.mrk = SwitchCmd(State::switch_id(sw_id),
                                            default_state.is_switch_straight(
                                                State::switch_id(sw_id)))});
              }
              return true;
            },
            [&](const TC::Cmd::RemoveTrains &) {
              tx_buf.push(
                  TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});
              return true;
            },
            [&](const TC::Cmd::RunTree &cmd) {
              spawn_tree_task(run_tree,
                              TC::Tree::Init{.loco_id   = cmd.id,
                                             .tree_type = cmd.tree_type,
                                             .value1    = cmd.value1,
                                             .value2    = cmd.value2,
                                             .value3    = cmd.value3,
                                             .state     = state});
              return true;
            },
            [&](const TC::Cmd::Quit &) {
              if (waiting_ui_update_worker_tid >= 0) {
                reply(waiting_ui_update_worker_tid, TC::Quit{});
                waiting_ui_update_worker_tid = -1;
              }
              if (waiting_can_tx_worker_tid >= 0) {
                reply(waiting_can_tx_worker_tid, TC::Quit{});
                waiting_can_tx_worker_tid = -1;
              }
              return true;
            },
            [&](const TC::Cmd::Nav &cmd) {
              if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX) {
                Debug_Puts(tx_tid, "Invalid node index in nav command");
                return false;
              }
              spawn_tree_task(
                  run_tree,
                  TC::Tree::Init{.loco_id   = cmd.id,
                                 .tree_type = TC::Tree::Type::NAVIGATE,
                                 .value1    = cmd.node_idx,
                                 .value2    = static_cast<int>(cmd.speed),
                                 .value3    = cmd.offset,
                                 .state     = state});
              return true;
            },
            [&](const TC::Cmd::Reg &cmd) {
              if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX) {
                Debug_Puts(tx_tid, "Invalid node index in reg command");
                return false;
              }
              if (TrainState *train = state.get_loco(cmd.id)) {
                train->inital_node_idx = cmd.node_idx;
                state.trains_dirty     = true;
              }
              return true;
            },
            [&](const TC::Cmd::Reserve &cmd) {
              if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX ||
                  (cmd.edge_dir != 0 && cmd.edge_dir != 1)) {
                Debug_Puts(tx_tid,
                           "Invalid node index or direction in reserve");
                return false;
              }

              // already reserved by another train
              auto res = track.get_reservation(cmd.node_idx, cmd.edge_dir);
              if (res != UNRESERVED && res != cmd.id) {
                return false;
              }
              Debug_Puts(tx_tid, "Res: ", track[cmd.node_idx].name,
                         cmd.edge_dir == 0 ? "S" : "C", "(this train: ", cmd.id,
                         ") prev ", res);
              track.reserve(cmd.node_idx, cmd.edge_dir, cmd.id);
              return true;
            },
            [&](const TC::Cmd::ReleaseReserve &cmd) {
              if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX ||
                  (cmd.edge_dir != 0 && cmd.edge_dir != 1)) {
                Debug_Puts(tx_tid,
                           "Invalid node index or direction in reserve path");
                return false;
              }

              // can't release if not reserved by this train
              if (auto res = track.get_reservation(cmd.node_idx, cmd.edge_dir);
                  res != UNRESERVED && res != cmd.id) {
                Debug_Puts(tx_tid, "Can't release reservation, reserved by ",
                           res, " (this train: ", cmd.id, ")");
                return false;
              }

              track.release(cmd.node_idx, cmd.edge_dir, cmd.id);
              return true;
            },
            [&](const TC::Cmd::Invalid &) { return false; },
        },
        command);
  }

  void handle(const int tid, const TC::RX &msg) {
    auto mrk = decode_frame(msg.frame);
    state.update_from_mrk(mrk, msg.time);
    simple_pacing_can_send = simple_pacing_can_send || (msg.frame.resp == 1);
    maybe_tx();
    publish_tree_update(TC::Tree::Update{.mrk = mrk, .time = msg.time});
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
    auto res = expand_user_command(msg);
    reply(tid, TC::Ack{.success = res});
    maybe_tx();
    if (std::get_if<TC::Cmd::Quit>(&msg)) {
      exit();
    }
  }

  void handle(const int tid, const TC::Tree::Ready &msg) {
    auto *mailbox = trees.get_ref(tid);
    if (mailbox == nullptr) {
      reply_with_error(tid);
      return;
    }
    auto loco = state.get_loco(msg.train.id);
    if (loco) {
      loco->ve_nm = msg.train.ve_nm;
      loco->v_max_umpt[msg.train.req_speed] =
          msg.train.v_max_umpt[msg.train.req_speed];
      loco->a_nmpt2[msg.train.req_speed] =
          msg.train.a_nmpt2[msg.train.req_speed];
      loco->target_node_idx = msg.train.target_node_idx;
      state.trains_dirty    = true;
    }

    auto next_msg = mailbox->msgs.pop();
    if (next_msg.has_value()) {
      reply(tid, next_msg.value());
      return;
    }

    mailbox->waiting = true;
  }

  void handle(const int tid, const TC::Tree::Tick &msg) {
    reply(tid, TC::Ack{});
    publish_tree_update(
        TC::Tree::Update{.mrk = UnknownCmd{}, .time = msg.time});
  }

  void handle(const int tid, const TC::Tree::Exit &) {
    trees.remove(tid);
    reply(tid, TC::Ack{});
  }

  template <class T> void handle(int sender_tid, const T &) {
    reply_with_error(sender_tid);
  }

public:
  static constexpr auto NAME                      = "TCSERVER";
  static constexpr auto TRACK                     = Track::Layout::A;
  static constexpr auto TICKS_BETWEEN_TRAIN_TICKS = 10;
  TrainControlServer() : track(TRACK) {
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