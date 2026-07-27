#pragma once

#include "buffer.h"
#include "clock_server.h"
#include "debug.h"
#include "map.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "pathfind.h"
#include "static_string.h"
#include "syscall.h"
#include "time.h"
#include "train_state.h"
#include "train_trees.h"
#include "uart_tx_server.h"
#include <cstddef>
#include <limits>
#include <optional>
#include <variant>

template <size_t TX_BUFFER_SIZE = 64> class TrainControlServer {
  int waiting_can_tx_worker_tid = -1;
  bool simple_pacing_can_send   = true;

  Track track;
  struct TreeMailbox {
    Buffer<TC::Tree::Msg, 16> msgs{};
    bool waiting = false;
  };

  Buffer<MRKCmd, TX_BUFFER_SIZE> tx_buf;
  Map<int, TreeMailbox, 10> trees;

  struct CalibratingTrain {
    uint32_t num   = 0;
    uint32_t speed = 0;
  } calibrating_train{};

  int cs_tid  = -1;
  int tx_tid  = -1;
  int web_tid = -1;
  TrackState state{};

  static int abs_int(int x) { return x < 0 ? -x : x; }

  std::optional<int> sensor_dist_on_path_um(const TrainState &train,
                                            int sensor_node_idx,
                                            bool require_reserved) {
    auto path    = train.e_path.decode(track);
    int dist_um  = 0;
    bool matched = false;

    for (const auto &node : path) {
      if (node.type == NODE_SENSOR && node.node_idx == sensor_node_idx) {
        if (!require_reserved || dist_um <= train.res_dist_um) {
          matched = true;
        }
        break;
      }
      dist_um += node.dx_next * 1000;
    }

    if (!matched) {
      return std::nullopt;
    }

    return dist_um;
  }

  std::optional<uint32_t> attribute_sensor_loco(const SensorData &sens) {
    int sensor_node_idx = sens.sid - 1;
    if (sensor_node_idx < 0 || sensor_node_idx >= TRACK_MAX) {
      return std::nullopt;
    }

    auto pick_best =
        [&](bool require_reserved,
            bool require_initial_for_unlocalized) -> std::optional<uint32_t> {
      int best_abs_dist = std::numeric_limits<int>::max();
      std::optional<uint32_t> best_loco{};

      for (const auto &train : state.trains) {
        if (require_initial_for_unlocalized && !train.last_sensor.has_value() &&
            train.inital_node_idx >= 0 &&
            train.inital_node_idx != sensor_node_idx) {
          continue;
        }

        auto sensor_dist_um =
            sensor_dist_on_path_um(train, sensor_node_idx, require_reserved);
        if (!sensor_dist_um.has_value()) {
          continue;
        }

        int abs_dist = abs_int(train.d_um - sensor_dist_um.value());
        if (abs_dist < best_abs_dist) {
          best_abs_dist = abs_dist;
          best_loco     = train.id;
        }
      }

      return best_loco;
    };

    // Primary policy: attribute only to trains that currently reserve the
    // sensor along their active path, selecting the closest by |d_um - dist|.
    if (auto reserved_pick = pick_best(true, false);
        reserved_pick.has_value()) {
      return reserved_pick;
    }

    // Bootstrap fallback: if nobody has the sensor reserved yet, allow a
    // nearest in-path attribution gated by initial-node constraints for
    // trains that are still unlocalized.
    return pick_best(false, true);
  }

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

  Message handle_command(const TC::Cmd::Light &cmd) {
    tx_buf.push(LightCmd(cmd.id, cmd.on));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Function &cmd) {
    tx_buf.push(FunctionCmd(cmd.id, cmd.function, cmd.value));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Speed &cmd) {
    tx_buf.push(SpeedCmd(cmd.id, user_speed_to_mrk_level(cmd.speed)));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Switch &cmd) {
    tx_buf.push(SwitchCmd(static_cast<uint16_t>(cmd.id), cmd.straight));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Reverse &cmd) {
    spawn_tree_task(run_tree, TC::Tree::Init{
                                  .loco_id   = cmd.id,
                                  .tree_type = TC::Tree::Type::REVERSE,
                                  .value1    = 0,
                                  .value2    = 0,
                                  .value3    = 0,
                                  .state     = state,
                              });
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Direction &cmd) {
    tx_buf.push(DirectionCmd(cmd.id, cmd.backward));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Stop &) {
    tx_buf.push(ControlCmd(ControlCmd::CMD_STOP));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Go &) {
    tx_buf.push(ControlCmd(ControlCmd::CMD_GO));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Reset &) {
    TrackState default_state{};

    tx_buf.push(ControlCmd(ControlCmd::CMD_REMOVE_TRAINS));
    tx_buf.push(ControlCmd(ControlCmd::CMD_GO));

    for (const TrainState &train : default_state.trains) {
      tx_buf.push(LightCmd(train.id, train.light_on));
      tx_buf.push(SpeedCmd(train.id, user_speed_to_mrk_level(train.req_speed)));
      tx_buf.push(DirectionCmd(train.id, train.backward));
    }

    for (uint32_t sw_id = 0; sw_id < 22; ++sw_id) {
      tx_buf.push(SwitchCmd(
          TrackState::switch_id(sw_id),
          default_state.is_switch_straight(TrackState::switch_id(sw_id))));
    }
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::RemoveTrains &) {
    tx_buf.push(ControlCmd(ControlCmd::CMD_REMOVE_TRAINS));
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::RunTree &cmd) {
    spawn_tree_task(run_tree, TC::Tree::Init{
                                  .loco_id   = cmd.id,
                                  .tree_type = cmd.tree_type,
                                  .value1    = cmd.value1,
                                  .value2    = cmd.value2,
                                  .value3    = cmd.value3,
                                  .state     = state,
                              });
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Quit &) {
    if (waiting_can_tx_worker_tid >= 0) {
      reply(waiting_can_tx_worker_tid, TC::Quit{});
      waiting_can_tx_worker_tid = -1;
    }
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Nav &cmd) {
    if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX) {
      return TC::Ack{};
    }
    spawn_tree_task(run_tree, TC::Tree::Init{
                                  .loco_id   = cmd.id,
                                  .tree_type = TC::Tree::Type::NAVIGATE,
                                  .value1    = cmd.node_idx,
                                  .value2    = static_cast<int>(cmd.speed),
                                  .value3    = cmd.offset,
                                  .state     = state,
                              });
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Reg &cmd) {
    if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX) {
      return TC::Ack{};
    }
    if (TrainState *train = state.get_loco(cmd.id)) {
      train->inital_node_idx = cmd.node_idx;
    }
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Reserve &cmd) {
    int res_dist_um = 0;

    for (const auto &node : cmd.path) {
      if (res_dist_um > cmd.lookahead_um) {
        break;
      }

      for (const auto &loco : state.trains) {
        if (loco.id == cmd.id) {
          continue;
        }

        Path other_path = loco.e_path.decode(track);

        // forwards
        {
          int node_d_um = other_path.dist_along_path(node) * 1000;
          if (node_d_um >= 0 &&
              (loco.d_um < node_d_um &&
               node_d_um <
                   std::max(loco.d_um + loco.stop_dist_um, loco.res_dist_um))) {
            state.get_loco(cmd.id)->res_dist_um = res_dist_um;
            return TC::Cmd::ReserveResponse{res_dist_um};
          }
        }

        // backwards
        {
          auto rev_path = other_path.reverse();
          int node_d_um = rev_path.dist_along_path(node) * 1000;
          if (node_d_um >= 0 &&
              std::min(rev_path.dist_mm * 1000 - loco.d_um - loco.stop_dist_um,
                       loco.res_dist_um) < node_d_um &&
              node_d_um < rev_path.dist_mm * 1000 - loco.d_um) {
            state.get_loco(cmd.id)->res_dist_um = res_dist_um;
            return TC::Cmd::ReserveResponse{res_dist_um};
          }
        }
      }

      res_dist_um += node.dx_next * 1000;
    }
    state.get_loco(cmd.id)->res_dist_um = res_dist_um;
    state.get_loco(cmd.id)->e_path      = cmd.path;
    return TC::Cmd::ReserveResponse{res_dist_um};
  }

  Message handle_command(const TC::Cmd::Invalid &) { return TC::Ack{}; }

  void handle(const int tid, const TC::RX &msg) {
    auto mrk = decode_frame(msg.frame);
    state.update(mrk);
    simple_pacing_can_send = simple_pacing_can_send || (msg.frame.resp == 1);
    maybe_tx();

    if (auto mrk_msg = std::get_if<SensorData>(&mrk); mrk_msg) {
      mrk_msg->loco_id = 0;
      if (auto loco_id = attribute_sensor_loco(*mrk_msg); loco_id.has_value()) {
        mrk_msg->loco_id = loco_id.value();
      }
    }

    publish_tree_update(TC::Tree::Update{
        .state = state,
        .mrk   = mrk,
        .time  = msg.time,
    });
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::UIReady &) {
    reply(tid, TC::UIUpdate{state});
  }

  void handle(const int tid, const TC::TXReady &) {
    waiting_can_tx_worker_tid = tid;
    maybe_tx();
  }

  void handle(const int tid, const TC::Cmd::Any &msg) {
    std::visit([&](auto &&arg) { reply(tid, handle_command(arg)); }, msg);
    maybe_tx();
    if (auto data = std::get_if<TC::Cmd::Quit>(&msg); data) {
      Debug_Puts(tx_tid, "train control server EXITING\n\r");
      exit();
    }
  }

  void handle(const int tid, const TC::Tree::Ready &msg) {
    auto *mailbox = trees.get_ref(tid);
    if (mailbox == nullptr) {
      reply_with_error(tid);
      return;
    }

    if (auto loco = state.get_loco(msg.train.id); loco) {
      *loco = msg.train;
    }

    auto next_msg_opt = mailbox->msgs.pop();
    if (!next_msg_opt.has_value()) {
      mailbox->waiting = true;
      return;
    }
    // need to update state to be the most recent one
    auto next_msg = next_msg_opt.value();
    std::visit(
        [&](auto &next_msg) {
          next_msg.state = state;
          reply(tid, next_msg);
        },
        next_msg);
  }

  void handle(const int tid, const TC::Tree::Tick &msg) {
    reply(tid, TC::Ack{});
    publish_tree_update(TC::Tree::Update{
        .state = state,
        .mrk   = UnknownCmd{},
        .time  = msg.time,
    });
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
  static constexpr auto TRACK                     = Track::Layout::B;
  static constexpr auto TICKS_BETWEEN_TRAIN_TICKS = 10;
  TrainControlServer() : track(TRACK) {
    auto response = RegisterAs(NAME);
    _assert(response == 0, "TC  REGISTERAS FAILED");

    cs_tid = WhoIs(ClockServer<>::NAME);
    _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

    tx_tid = WhoIs(UART_TX_Server::NAME);
    _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

    web_tid = WhoIs(UART03_TX_Server::NAME);
    _assert(web_tid >= 0, "TX SERVER3 WHOIS FAILED");

    create(1, rx_can_worker);
    create(2, tx_can_worker);
    create(5, train_tick_worker);

    handle_command(TC::Cmd::RemoveTrains{});
    handle_command(TC::Cmd::Reset{});
  }

  void run() {
    int sender_tid;
    Message msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  }
};
