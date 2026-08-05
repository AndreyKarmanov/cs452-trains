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
#include "track_node.h"
#include "train_state.h"
#include "train_trees.h"
#include "uart_tx_server.h"
#include <cstddef>
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
    spawn_tree_task(run_tree, TC::Tree::Init{.loco_id   = cmd.id,
                                             .tree_type = cmd.tree_type,
                                             .value1    = cmd.value1,
                                             .value2    = cmd.value2,
                                             .value3    = cmd.value3,
                                             .state     = state});
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
      return TC::Ack{.return_code = -1};
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
      return TC::Ack{.return_code = -1};
    }
    if (auto train = state.get_loco(cmd.id); train && train->e_path.empty()) {
      train->e_path.push(cmd.node_idx);
    }
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Reserve &cmd) {
    if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX) {
      return TC::Ack{.return_code = -1};
    }

    // already reserved by another train
    auto res = track.get_reservation(cmd.node_idx);
    if (res != UNRESERVED && res != cmd.id) {
      return TC::Ack{.return_code = -1};
    }
    track.reserve(cmd.node_idx, cmd.id);
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::ReleaseReserve &cmd) {
    if (cmd.node_idx < 0 || cmd.node_idx >= TRACK_MAX) {
      return TC::Ack{.return_code = -1};
    }

    // can't release if not reserved by this train
    auto res = track.get_reservation(cmd.node_idx);
    if (res != UNRESERVED && res != cmd.id) {
      return TC::Ack{.return_code = -1};
    }
    track.release(cmd.node_idx, cmd.id);
    return TC::Ack{};
  }

  Message handle_command(const TC::Cmd::Invalid &) {
    return TC::Ack{.return_code = -1};
  }

  Message handle_command(const TC::Cmd::FindPath &cmd) {
    if (cmd.start_idx < 0 || cmd.start_idx >= TRACK_MAX || cmd.goal_idx < 0 ||
        cmd.goal_idx >= TRACK_MAX) {
      return TC::PathReply{.return_code = -2, .path = EncodedPath{}};
    }

    auto res = track.find_path(cmd.start_idx, cmd.goal_idx, cmd.allow_reverse,
                               false, cmd.id);
    if (!res.has_value()) {
      return TC::PathReply{.return_code = -1, .path = EncodedPath()};
    }
    return TC::PathReply{.return_code = 0, .path = EncodedPath(res.value())};
  }

  void handle(const int tid, const TC::RX &msg) {
    auto mrk = decode_frame(msg.frame);
    state.update(mrk);
    simple_pacing_can_send = simple_pacing_can_send || (msg.frame.resp == 1);
    maybe_tx();
    publish_tree_update(TC::Tree::Update{
        .state = state,
        .mrk   = mrk,
        .time  = msg.time,
    });
    reply(tid, TC::Ack{});
  }

  void handle(const int tid, const TC::UIReady &) {
    state.reservations.clear();
    for (int node_idx = 0; node_idx < TRACK_MAX; ++node_idx) {
      auto res = track.get_reservation(node_idx);
      if (res != UNRESERVED) {
        state.reservations.set(node_idx, res);
      }
    }

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
