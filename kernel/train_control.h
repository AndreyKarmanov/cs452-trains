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
#include "train_state.h"
#include "train_trees.h"
#include "uart_tx_server.h"
#include <cstddef>

template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  int waiting_ui_update_worker_tid = -1;
  int waiting_can_tx_worker_tid    = -1;
  bool simple_pacing_can_send      = true;

  static constexpr auto TRACK = Track::Layout::A;
  Track pathfind;

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

  void expand_user_command(const TC::Cmd::Any &command) {
    std::visit(
        Overloaded{
            [&](const TC::Cmd::Light &cmd) {
              tx_buf.push(TC::TX{.mrk = LightCmd(cmd.id, cmd.on)});
            },
            [&](const TC::Cmd::Function &cmd) {
              tx_buf.push(
                  TC::TX{.mrk = FunctionCmd(cmd.id, cmd.function, cmd.value)});
            },
            [&](const TC::Cmd::Speed &cmd) {
              tx_buf.push(TC::TX{
                  .mrk = SpeedCmd(cmd.id, user_speed_to_mrk_level(cmd.value))});
            },
            [&](const TC::Cmd::Switch &cmd) {
              tx_buf.push(TC::TX{.mrk = SwitchCmd(static_cast<uint16_t>(cmd.id),
                                                  cmd.straight)});
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
            },
            [&](const TC::Cmd::Direction &cmd) {
              tx_buf.push(TC::TX{.mrk = DirectionCmd(cmd.id, cmd.backward)});
            },
            [&](const TC::Cmd::Stop &) {
              tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_STOP)});
            },
            [&](const TC::Cmd::Go &) {
              tx_buf.push(TC::TX{.mrk = ControlCmd(ControlCmd::CMD_GO)});
            },
            [&](const TC::Cmd::Reset &) {
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
            },
            [&](const TC::Cmd::RemoveTrains &) {
              tx_buf.push(
                  TC::TX{.mrk = ControlCmd(ControlCmd::CMD_REMOVE_TRAINS)});
            },
            [&](const TC::Cmd::RunTree &cmd) {
              spawn_tree_task(run_tree,
                              TC::Tree::Init{.loco_id   = cmd.id,
                                             .tree_type = cmd.tree_type,
                                             .value1    = cmd.value1,
                                             .value2    = cmd.value2,
                                             .value3    = cmd.value3,
                                             .state     = state});
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
            },
            [&](const TC::Cmd::Nav &cmd) {
              auto node_idx = pathfind.get_idx(cmd.to.c_str());
              if (!node_idx.has_value()) {
                Debug_Puts(tx_tid, "Invalid name in nav command");
                return;
              }
              spawn_tree_task(
                  run_tree,
                  TC::Tree::Init{.loco_id   = cmd.id,
                                 .tree_type = TC::Tree::Type::NAVIGATE,
                                 .value1    = node_idx.value(),
                                 .value2    = static_cast<int>(cmd.speed),
                                 .value3    = cmd.offset,
                                 .state     = state});
            },
            [&](const TC::Cmd::Reg &cmd) {
              if (TrainState *train = state.get_loco(cmd.id)) {
                auto node_idx = pathfind.get_idx(cmd.sensor.c_str());
                if (!node_idx.has_value()) {
                  Debug_Puts(tx_tid, "Invalid sensor name in reg command");
                  return;
                }
                train->inital_node_idx = node_idx.value();
                state.trains_dirty     = true;
              }
            },
            [&](const TC::Cmd::Reserve &cmd) {
              auto msg    = TC::Tree::TrackReserved{};
              msg.loco_id = cmd.id;
              for (auto &node : cmd.path) {
                if (node.node_idx < 0 || node.node_idx >= TRACK_MAX ||
                    (node.dir != 0 && node.dir != 1)) {
                  Debug_Puts(tx_tid,
                             "Invalid node index or direction in reserve path");
                  break;
                }

                auto edge = pathfind[node.node_idx].edge[node.dir];

                if (edge.reservation != UNRESERVED &&
                    static_cast<uint32_t>(edge.reservation) != cmd.id) {
                  break;
                }

                pathfind.reserve(node.node_idx, node.dir, cmd.id);
                msg.path.push(node);
              }
              publish_tree_update(msg);
            },
            [&](const TC::Cmd::ReleaseReserve &cmd) {
              for (auto &node : cmd.path) {
                if (node.node_idx < 0 || node.node_idx >= TRACK_MAX ||
                    (node.dir != 0 && node.dir != 1)) {
                  Debug_Puts(tx_tid,
                             "Invalid node index or direction in reserve path");
                  break;
                }
                pathfind.release(node.node_idx, node.dir, cmd.id);
              }
            },
            [&](const TC::Cmd::Invalid &) { return; },
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
    expand_user_command(msg);
    maybe_tx();
    reply(tid, TC::Ack{});
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
    auto loco = state.get_loco(msg.train.loco_id);
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
  static constexpr auto TICKS_BETWEEN_TRAIN_TICKS = 10;
  TrainControlServer() : pathfind(TRACK) {
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