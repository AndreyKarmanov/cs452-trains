#include "train_server.h"
#include "behaviour_tree.h"
#include "io_helpers.h"
#include "message.h"
#include "pathfind.h"
#include "train_control.h"

namespace {

  struct LogNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      Puts(bb.txs_tid, "tree tick for loco ", bb.loco_id, " value ",
           bb.req_speed, "\n\r");
      return NodeResult::Success;
    }
  };

  struct RepeatForeverNode : public DecoratorNode {
    NodeResult tick(Blackboard &bb) override {
      while (true) {
        NodeResult result = child->tick(bb);
        if (result == NodeResult::Failure) {
          return NodeResult::Failure;
        } else {
          return NodeResult::Running;
        }
      }
      return NodeResult::Success;
    }
  };

  struct SetSpeedNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (bb.txs_tid < 0) {
        return NodeResult::Failure;
      }

      if (bb.req_speed == bb.state.get_loco(bb.loco_id).requested_speed) {
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(
          bb.tcs_tid, TC::Cmd::Speed{.id = bb.loco_id, .value = bb.req_speed});
      if (!resp.has_value()) {
        return NodeResult::Failure;
      }
      return NodeResult::Success;
    }
  };

  template <size_t N> struct ExpectPathNode : public LeafNode {
    std::array<uint16_t, N> path;
    size_t path_idx = 0;
    Pathfind pathfind{'b'};
    uint32_t last_sensor_passed_tick = 0;

    ExpectPathNode(std::array<uint16_t, N> expected_path)
        : path(expected_path) {}

    NodeResult tick(Blackboard &bb) override {
      if (std::get_if<SensorData>(&bb.new_event)) {
        const SensorData &data = std::get<SensorData>(bb.new_event);
        if (data.new_state == 0) {
          return NodeResult::Running;
        }
        if (data.sensor_id == path[path_idx]) {
          if (path_idx >= 1) {
            auto distance = pathfind.shortest_path(path.at(path_idx - 1) - 1,
                                                   path.at(path_idx) - 1);
            if (distance.has_value()) {
              bb.est_speed = (distance.value().dist * 10) /
                             (bb.event_tick - last_sensor_passed_tick);
            }
          }
          path_idx++;
          Debug_Puts(bb.txs_tid, "Path: ", path_idx, "/", N, " speed ",
                     bb.est_speed / 10, ".", bb.est_speed % 10, "mm / tick ",
                     bb.event_tick - last_sensor_passed_tick,
                     " Tick delta\n\r");
          last_sensor_passed_tick = bb.event_tick;
          if (path_idx == N) {
            Debug_Puts(bb.txs_tid, "path completed successfully\n\r");
            return NodeResult::Success;
          }
        } else {
          bb.error_msg.set("Unexpected sid: ", data.sensor_id, " expected ",
                           path[path_idx]);
          return NodeResult::Failure;
        }
      } else if (path_idx == N) {
        return NodeResult::Success;
      }
      return NodeResult::Running;
    }
  };
} // namespace

void train_tree_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);
  auto cs_tid  = WhoIs(ClockServer<>::NAME);

  Blackboard bb{};
  bb.tcs_tid = tcs_tid;
  bb.txs_tid = tx_tid;
  bb.cs_tid  = cs_tid;

  auto sid = [](char b, int n) -> uint16_t { return (b - 'A') * 16 + n; };

  // B6 C12 A4 B16 C10 B1 D14 E14 E9 D5 E6 D4
  std::array<uint16_t, 12> path{
      sid('B', 6),  sid('C', 12), sid('A', 4),  sid('B', 16),
      sid('C', 10), sid('B', 1),  sid('D', 14), sid('E', 14),
      sid('E', 9),  sid('D', 5),  sid('E', 6),  sid('D', 4),
  };

  SequenceNode tree{};

  SetSpeedNode set_speed_node{};
  tree.children.push(&set_speed_node);

  FallBackNode fallback_node{};

  ExpectPathNode expect_path_node(path);
  fallback_node.children.push(&expect_path_node);

  while (true) {
    auto next_msg = send<TC::TreeMsg>(tcs_tid, TC::TreeReady{});
    if (!next_msg.has_value()) {
      break;
    }

    std::visit(
        [&](auto &&event) {
          using Event = std::decay_t<decltype(event)>;

          if constexpr (std::is_same_v<Event, TC::InitTree>) {
            bb.loco_id   = event.loco_id;
            bb.req_speed = static_cast<uint16_t>(event.value);
          } else if constexpr (std::is_same_v<Event, TC::TreeUpdate>) {
            bb.state.update_from_mrk(event.mrk);
            bb.new_event  = event.mrk;
            bb.event_tick = event.time;
          }
        },
        next_msg.value());
    auto result = tree.tick(bb);
    if (result == NodeResult::Failure) {
      Debug_Puts(bb.txs_tid, "tree failed\n\r");
      Debug_Puts(bb.txs_tid, "Error: ", bb.error_msg, "\n\r");
      break;
    } else if (result == NodeResult::Success) {
      Debug_Puts(bb.txs_tid, "tree succeeded\n\r");
      break;
    }
  }
  std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
}