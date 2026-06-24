#include "train_server.h"
#include "behaviour_tree.h"
#include "io_helpers.h"
#include "message.h"
#include "mrk.h"
#include "pathfind.h"
#include "train_control.h"
#include <algorithm>
#include <cstdint>
#include <numeric>

namespace {
  struct DebugPrintPath : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      StaticString<128> path_str{};
      path_str.append("Path: ");
      for (auto node : bb.path) {
        path_str.append(bb.pathfinder.track[node.node_idx].name, " ",
                        node.distance_to_next_node, "mm -> ");
      }
      Debug_Puts(bb.txs_tid, path_str);
      return NodeResult::Success;
    }
  };

  struct SetSpeedNode : public LeafNode {
    uint16_t req_speed;
    bool set_speed = false;

    SetSpeedNode(uint16_t speed) : req_speed(speed) {}
    NodeResult tick(Blackboard &bb) override {
      if (bb.txs_tid < 0) {
        return NodeResult::Failure;
      }

      if (set_speed ||
          req_speed == bb.state.get_loco(bb.loco_id).requested_speed) {
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(
          bb.tcs_tid, TC::Cmd::Speed{.id = bb.loco_id, .value = req_speed});
      if (!resp.has_value()) {
        return NodeResult::Failure;
      }
      set_speed = true;
      return NodeResult::Success;
    }
  };

  struct SaveSensorNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1) {
        if (bb.seen_sensors.size() == bb.seen_sensors.capacity()) {
          bb.seen_sensors.pop();
        }

        bb.seen_sensors.push({
            .sid  = data->sensor_id,
            .tick = bb.event_tick,
        });
      }
      return NodeResult::Success;
    }
  };

  struct AwaitSensorNode : public LeafNode {
    int sensor_id     = -1;
    AwaitSensorNode() = default;
    AwaitSensorNode(int sensor_id) : sensor_id(sensor_id) {}
    NodeResult tick(Blackboard &bb) override {
      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1 &&
          (data->sensor_id == sensor_id || sensor_id == -1)) {
        return NodeResult::Success;
      }
      return NodeResult::Running;
    }
  };

  struct PathLocalizerNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        return NodeResult::Success;
      } else if (auto data = std::get_if<SensorData>(&bb.new_event);
                 data && data->new_state == 1) {

        auto sensor_id_cmp = [&](PathNode &node) {
          if (node.type == NODE_SENSOR) {
            return node.node_idx + 1;
          }
          return -1;
        };

        auto idx = std::ranges::find(bb.path, data->sensor_id, sensor_id_cmp);
        if (idx == bb.path.end()) {
          Debug_Puts(bb.txs_tid, "Couldn't find self in path\n\r");
          return NodeResult::Failure;
        }

        if (bb.travelled_dist.size() == bb.travelled_dist.capacity()) {
          bb.travelled_dist.pop();
        }
        bb.travelled_dist.push({
            .distance = std::accumulate(bb.path.begin(), idx + 1, uint16_t(0),
                                        [](uint16_t acc, const PathNode &node) {
                                          return acc +
                                                 node.distance_to_prev_node;
                                        }),
            .tick     = bb.event_tick,
        });
        auto skipped_nodes = std::distance(bb.path.begin(), idx) + 1;
        bb.path.pop(skipped_nodes);
      }
      return NodeResult::Success;
    }
  };

  struct StopAtDistance : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (bb.path.size() <= 1 ||
          bb.state.get_loco(bb.loco_id).requested_speed == 0) {
        return NodeResult::Success;
      }

      auto total_dist = std::accumulate(
          bb.path.begin(), bb.path.end(), 0, [](int acc, const PathNode &node) {
            return acc + node.distance_to_next_node;
          });

      if (total_dist <= bb.stop_distance) {
        Debug_Puts(bb.txs_tid, "Stopping ", bb.loco_id, " stop dist ",
                   bb.stop_distance, " at distance ", total_dist, "mm\n\r");
        auto res = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Speed(bb.loco_id, 0));
        if (!res.has_value()) {
          return NodeResult::Failure;
        }
        return NodeResult::Success;
      }
      return NodeResult::Running;
    };
  };

  struct StopAtDonePath : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (bb.path.empty()) {
        auto res = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Speed(bb.loco_id, 0));
        if (!res.has_value()) {
          return NodeResult::Failure;
        }
        return NodeResult::Success;
      }
      return NodeResult::Running;
    };
  };

  auto sid = [](char b, int n) -> uint16_t { return (b - 'A') * 16 + n; };

  // sets the train to move slowly (speed 4)
  struct LocalizerTree : public LeafNode {
    SequenceNode tree{};
    SetSpeedNode set_speed{4};
    AwaitSensorNode await_sensor{};
    SetSpeedNode zero_speed{0};

    LocalizerTree() {
      tree.children.push(&set_speed);
      tree.children.push(&await_sensor);
      tree.children.push(&zero_speed);
    }

    NodeResult tick(Blackboard &bb) override {
      if (!bb.seen_sensors.empty()) {
        return NodeResult::Success;
      }
      return tree.tick(bb);
    }
  };

  struct GoToNode : public LeafNode {
    bool path_initalized = false;

    NodeResult tick(Blackboard &bb) override {
      if (path_initalized) {
        return NodeResult::Success;
      }

      if (bb.seen_sensors.empty()) {
        bb.error_msg = "Failed to find start";
        return NodeResult::Failure;
      }

      auto goal_idx = bb.pathfinder.get_idx("B6");
      if (!goal_idx.has_value()) {
        bb.error_msg = "Failed to find goal";
        return NodeResult::Failure;
      }

      auto path_opt = bb.pathfinder.shortest_path(
          bb.seen_sensors.peek_last()->sid - 1, goal_idx.value());

      if (!path_opt.has_value()) {
        bb.error_msg = "Failed to find path";
        return NodeResult::Failure;
      }
      bb.path         = path_opt.value();
      path_initalized = true;
      return NodeResult::Success;
    }
  };

  struct AddLoop : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (bb.path.empty()) {
        bb.error_msg = "No path to loop";
        return NodeResult::Failure;
      }

      if (bb.path.peek()->node_idx == bb.path.peek_last()->node_idx) {
        return NodeResult::Success;
      }

      auto path_opt = bb.pathfinder.shortest_path(bb.path.peek_last()->node_idx,
                                                  bb.path.peek()->node_idx);

      if (!path_opt.has_value()) {
        bb.error_msg = "Failed to find path";
        return NodeResult::Failure;
      }
      bb.path = bb.path + path_opt.value();
      return NodeResult::Success;
    }
  };

  struct PathFollower : public TreeNode {
    FallBackNode tree{};

    SequenceNode seq{};

    SaveSensorNode save_sensor{};
    LocalizerTree localizer_tree{};

    GoToNode create_loop_start_node{};
    DebugPrintPath debug_print{};
    RepeatNode debug_print_path{&debug_print, 1};

    AddLoop add_loop{};
    SetSpeedNode max_speed{14};

    PathLocalizerNode path_localizer{};
    StopAtDonePath stop_on_finish_path{};
    AwaitSensorNode await_sensor_node{sid('B', 6)};
    RepeatNode repeat_node{&await_sensor_node, 3};
    SaveSensorNode save_sensor_node{};

    SetSpeedNode zero_speed{0};
    InvertNode invert_zero_speed{&zero_speed};

    PathFollower() {
      // default always
      seq.children.push(&save_sensor);

      // localize if nothing is saved
      seq.children.push(&localizer_tree);

      // try to run the stop distance thing
      seq.children.push(&create_loop_start_node);
      seq.children.push(&add_loop);
      seq.children.push(&debug_print_path);
      seq.children.push(&max_speed);
      seq.children.push(&path_localizer);
      seq.children.push(&repeat_node);
      seq.children.push(&zero_speed);

      // set to zero speed and print failure
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

  Blackboard bb{
      .pathfinder{'a'},
  };
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
            bb.state         = event.state;
            bb.loco_id       = event.loco_id;
            bb.stop_distance = static_cast<uint16_t>(event.value);
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