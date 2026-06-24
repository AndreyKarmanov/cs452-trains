#include "train_server.h"
#include "behaviour_tree.h"
#include "io_helpers.h"
#include "message.h"
#include "pathfind.h"
#include "train_control.h"
#include <array>

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
    uint16_t req_speed;
    SetSpeedNode(uint16_t speed) : req_speed(speed) {}
    NodeResult tick(Blackboard &bb) override {
      if (bb.txs_tid < 0) {
        return NodeResult::Failure;
      }

      if (req_speed == bb.state.get_loco(bb.loco_id).requested_speed) {
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(
          bb.tcs_tid, TC::Cmd::Speed{.id = bb.loco_id, .value = req_speed});
      if (!resp.has_value()) {
        return NodeResult::Failure;
      }
      return NodeResult::Success;
    }
  };

  struct TracePathNode : public LeafNode {
    Pathfind pathfind;
    uint32_t last_sensor_passed_tick = 0;

    TracePathNode(const char track_layout) : pathfind(track_layout) {}

    NodeResult tick(Blackboard &bb) override {
      if (bb.path.is_empty()) {
        return NodeResult::Success;
      } else if (std::get_if<SensorData>(&bb.new_event)) {
        const SensorData &data = std::get<SensorData>(bb.new_event);
        if (data.new_state == 0) {
          return NodeResult::Running;
        }
        if (data.sensor_id == bb.path.peek().value()) {
          if (bb.last_seen_sensor != 0 &&
              bb.last_seen_sensor != data.sensor_id) {
            auto distance = pathfind.shortest_path(bb.last_seen_sensor - 1,
                                                   bb.path.peek().value() - 1);
            if (distance.has_value()) {
              bb.est_speed = (distance.value().dist * 10) /
                             (bb.event_tick - bb.last_seen_sensor_tick);
            }
          }

          Debug_Puts(bb.txs_tid, "Nodes left: ", bb.path.size(), " speed ",
                     bb.est_speed / 10, ".", bb.est_speed % 10, "mm / tick ",
                     bb.event_tick - bb.last_seen_sensor_tick,
                     " Tick delta\n\r");
          bb.last_seen_sensor_tick = bb.event_tick;
          bb.last_seen_sensor      = bb.path.pop().value();
          if (bb.path.is_empty()) {
            Debug_Puts(bb.txs_tid, "path completed successfully\n\r");
            return NodeResult::Success;
          }
        } else {
          Debug_Puts(bb.txs_tid, "Unexpected sid: ", data.sensor_id,
                     " expected ", bb.path.peek().value(), "\n\r");
          return NodeResult::Failure;
        }
      }
      return NodeResult::Running;
    }
  };

  struct SaveSensorNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (std::get_if<SensorData>(&bb.new_event)) {
        const SensorData &data = std::get<SensorData>(bb.new_event);
        if (data.new_state == 0) {
          return NodeResult::Running;
        }
        bb.last_seen_sensor      = data.sensor_id;
        bb.last_seen_sensor_tick = bb.event_tick;
        Debug_Puts(bb.txs_tid, "Saved sensor: ", bb.last_seen_sensor, "\n\r");
        return NodeResult::Success;
      }
      return NodeResult::Running;
    }
  };

  auto sid = [](char b, int n) -> uint16_t { return (b - 'A') * 16 + n; };

  // sets the train to move slowly (speed 4)
  struct LocalizerTree : public LeafNode {
    SequenceNode tree{};
    SetSpeedNode set_speed{4};
    SaveSensorNode save_sensor{};
    SetSpeedNode zero_speed{0};

    LocalizerTree() {
      tree.children.push(&set_speed);
      tree.children.push(&save_sensor);
      tree.children.push(&zero_speed);
    }

    NodeResult tick(Blackboard &bb) override {
      if (bb.last_seen_sensor != 0) {
        return NodeResult::Success;
      }
      return tree.tick(bb);
    }
  };

  struct CreateLoopStartNode : public LeafNode {
    Pathfind pathfind;
    CreateLoopStartNode(const char track_layout) : pathfind(track_layout) {}

    NodeResult tick(Blackboard &bb) override {

      if (bb.path_initialized) {
        return NodeResult::Success;
      }

      if (bb.last_seen_sensor == sid('B', 6)) {
        bb.path_initialized = true;
        return NodeResult::Success;
      }

      auto goal_idx = pathfind.get_idx("B6");
      if (bb.last_seen_sensor == 0 || !goal_idx.has_value()) {
        bb.error_msg = "Failed to find start or goal node";
        return NodeResult::Failure;
      }

      auto path =
          pathfind.shortest_path(bb.last_seen_sensor - 1, goal_idx.value());

      if (!path.has_value()) {
        bb.error_msg = "Failed to find path from to";
        return NodeResult::Failure;
      }

      StaticString<32> path_str{};
      path_str.append("Path: ");
      for (size_t i = 0; i < path->len; ++i) {
        if (pathfind.track[path->nodes[i]].type == NODE_SENSOR) {
          bb.path.push(path->nodes[i] + 1);
        }
      }; // remove the first sesnor since we already passed it

      bb.path_initialized = true;

      for (int i : bb.path) {
        path_str.append(pathfind.track[i - 1].name, " ");
      }

      Debug_Puts(bb.txs_tid, path_str);
      return NodeResult::Success;
    }
  };

  struct PathFollower : public TreeNode {
    // B6 C12 A4 B16 C10 B1 D14 E14 E9 D5 E6 D4
    // std::array<uint16_t, 12> path{
    //     sid('B', 6),  sid('C', 12), sid('A', 4),  sid('B', 16),
    //     sid('C', 10), sid('B', 1),  sid('D', 14), sid('E', 14),
    //     sid('E', 9),  sid('D', 5),  sid('E', 6),  sid('D', 4),
    // };

    FallBackNode tree{};

    SequenceNode seq{};
    LocalizerTree localizer_tree{};
    CreateLoopStartNode create_loop_start_node{'b'};
    SetSpeedNode max_speed{14};

    TracePathNode expect_path_node{'b'};
    SaveSensorNode save_sensor_node{};

    SetSpeedNode zero_speed{0};
    InvertNode invert_zero_speed{&zero_speed};

    PathFollower() {
      seq.children.push(&localizer_tree);
      seq.children.push(&create_loop_start_node);
      seq.children.push(&max_speed);
      seq.children.push(&expect_path_node);
      seq.children.push(&zero_speed);

      tree.children.push(&seq);
      tree.children.push(&invert_zero_speed);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
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

  PathFollower tree{};

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