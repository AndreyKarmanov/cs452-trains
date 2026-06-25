
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
  auto sid = [](char b, int n) -> uint16_t { return (b - 'A') * 16 + n; };
  constexpr int LOOP_START_SID = sid('E', 6);
  constexpr int LOOP_END_SID   = sid('D', 5);

  struct DebugPrintPath : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      StaticString<128> path_str{};
      path_str.append("Path: ", bb.path.size(), " ");
      for (auto node : bb.path) {
        path_str.append(bb.pathfinder.track[node.node_idx].name, " ");
      }
      Debug_Puts(bb.txs_tid, path_str);
      return NodeResult::Success;
    }
  };

  struct PrintSteadyStateSpeed : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      auto measurements_to_use = size_t{12};

      if (bb.dists.size() < measurements_to_use) {
        bb.error_msg = "NOt enough measurments";
        return NodeResult::Failure;
      }

      bool found_start      = false;
      int loops_to_use      = 2;
      int measurements_used = 0;

      uint32_t last_tick  = 0;
      uint32_t first_tick = 0;
      uint32_t total_dist = 0;

      for (auto it = bb.dists.end() - 1;
           it != bb.dists.begin() && loops_to_use > 0; it--) {
        if ((*it).sensor_data.sensor_id == LOOP_START_SID) {
          if (found_start == false) {
            last_tick   = (*it).tick;
            found_start = true;
          } else {
            loops_to_use--;
          }
        }
        if (loops_to_use == 0) {
          first_tick = (*it).tick;
        } else if (found_start) {
          total_dist += (*it).distance;
          measurements_used++;
        }
      }

      auto total_ticks     = last_tick - first_tick;
      auto estimated_speed = (total_dist * 1000) / total_ticks;

      Debug_Puts(bb.txs_tid, "Speed",
                 bb.state.get_loco(bb.loco_id).requested_speed,
                 "Total dist: ", total_dist, "mm, total ticks: ", total_ticks,
                 " speed: ", estimated_speed,
                 "tmm/tick Sensors used: ", measurements_used, "\n\r");

      return NodeResult::Success;
    }
  };

  struct PrintStoppingDistance : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      auto loco               = bb.state.get_loco(bb.loco_id);
      auto b                  = loco.top_speed[loco.requested_speed];
      auto measured_dist_hmm  = (bb.event_tick - bb.last_checkpoint) * b;
      auto stopped_sensor_cmd = LOOP_START_SID;

      uint16_t total_dist = 0;
      for (auto it = bb.dists.end() - 1; it != bb.dists.begin(); --it) {
        if ((*it).sensor_data.sensor_id == stopped_sensor_cmd) {
          break;
        }
        total_dist += (*it).distance;
      }
      auto stopping = (total_dist * 100 - measured_dist_hmm) / 100;

      Debug_Puts(bb.txs_tid, "Measured: ", measured_dist_hmm,
                 "hmm, Stopping: ", stopping, "mm b", b, " last checkpoint ",
                 bb.event_tick - bb.last_checkpoint, " ticks ago ",
                 "speed: ", loco.requested_speed, " b ", b, "\n\r");

      return NodeResult::Success;
    }
  };

  struct SaveCheckpointNode : public LeafNode {
    bool saved = false;
    NodeResult tick(Blackboard &bb) override {
      if (!saved) {
        bb.last_checkpoint = bb.event_tick;
        saved              = true;
      }
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

  struct SetTargetSpeedNode : public LeafNode {
    uint16_t req_speed;
    bool set_speed;
    SetTargetSpeedNode(uint16_t speed) : req_speed(speed) {}
    NodeResult tick(Blackboard &bb) override {
      if (bb.txs_tid < 0) {
        return NodeResult::Failure;
      }
      req_speed = bb.target_speed;

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

  struct SetDirectionNode : public LeafNode {
    bool backward;
    SetDirectionNode(bool backward) : backward(backward) {}
    NodeResult tick(Blackboard &bb) override {
      if (bb.txs_tid < 0) {
        return NodeResult::Failure;
      }

      if (backward == bb.state.get_loco(bb.loco_id).backward) {
        return NodeResult::Success;
      }

      auto resp =
          send<TC::Ack>(bb.tcs_tid, TC::Cmd::Direction{.id       = bb.loco_id,
                                                       .backward = backward});
      if (!resp.has_value()) {
        return NodeResult::Failure;
      }
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

        if (bb.dists.size() == bb.dists.capacity()) {
          bb.dists.pop();
        }
        bb.dists.push({
            .distance = std::accumulate(bb.path.begin(), idx + 1, uint16_t(0),
                                        [](uint16_t acc, const PathNode &node) {
                                          return acc +
                                                 node.distance_to_prev_node;
                                        }),
            .tick     = bb.event_tick,
            .sensor_data = *data,
        });
        auto skipped_nodes = std::distance(bb.path.begin(), idx) + 1;
        bb.path.pop(skipped_nodes);
      }
      return NodeResult::Success;
    }
  };

  struct PathLookaheadNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        bb.prev_lookahead_node = {.node_idx = -1};
        return NodeResult::Success;
      }

      // calculate distance travelled given current velocity
      // safe estimate is max velocity for speed
      // then, calculate distance based on velocity. suppose distance is 500
      int lookahead = 500;

      auto total_dist = 0;
      for (auto &node : bb.path) {
        total_dist += node.distance_to_prev_node;
        if (total_dist > lookahead) {
          break;
        }

        if (node.type == NODE_BRANCH) {
          if (node.should_br_be_curved &&
              bb.state.is_switch_straight(node.num)) {
            auto res =
                send<TC::Ack>(bb.tcs_tid, TC::Cmd::Switch(node.num, false));
            if (!res.has_value()) {
              return NodeResult::Failure;
            }
          }
        }
      }
      return NodeResult::Success;

      // lookahead to nodes within the next 500
      // assumption that dist 500 is within 20 nodes.
      PathNode node_buffer[20];
      int count = bb.path.lookahead(lookahead, node_buffer, 20);

      // index prev path node
      int prev_lookahead_node = 0;
      for (int i = 0; i < count; i++) {
        if (node_buffer[i] == bb.prev_lookahead_node) {
          prev_lookahead_node = i;
          break;
        }
      }

      // process all subsequent lookahead nodes
      for (int i = prev_lookahead_node + 1; i < count; i++) {
        Debug_Puts(bb.txs_tid, "Lookahead process for node: ",
                   bb.pathfinder.node_name(node_buffer[i].node_idx), "\n\r");
        // process lookahead here
        // TODO

        // update prev path node
        bb.prev_lookahead_node = node_buffer[i];
      }
      return NodeResult::Success;
    }
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

  // sets the train to move slowly (speed 4)
  struct InitalLocalizeTree : public LeafNode {
    SequenceNode tree{};
    SetSpeedNode set_speed{4};
    AwaitSensorNode await_sensor{};
    SetSpeedNode zero_speed{0};

    InitalLocalizeTree() {
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

  struct PathToNode : public LeafNode {
    bool path_initalized = false;

    NodeResult tick(Blackboard &bb) override {
      if (path_initalized) {
        return NodeResult::Success;
      }

      if (bb.seen_sensors.empty() && bb.path.empty()) {
        bb.error_msg = "Failed to find start";
        return NodeResult::Failure;
      }

      auto start_idx =
          bb.seen_sensors.peek_last().has_value()
              ? bb.seen_sensors.peek_last()->sid - 1 // sid -1 is it's node_idx
              : bb.path.peek_last().value().node_idx;

      auto goal_idx = bb.pathfinder.get_idx("B6");
      if (!goal_idx.has_value()) {
        bb.error_msg = "Failed to find goal";
        return NodeResult::Failure;
      }

      auto path_opt = bb.pathfinder.shortest_path(start_idx, goal_idx.value());

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

  struct StoppingDistance : public LeafNode {
    FallBackNode tree{};

    SequenceNode seq{};
    SequenceNode run_loop{};

    SaveSensorNode save_sensor{};
    InitalLocalizeTree localizer_tree{};

    PathToNode create_loop_start_node{};
    DebugPrintPath debug_print{};
    RepeatNode debug_print_path{&debug_print, 1};

    AddLoop loop_path{};
    SetTargetSpeedNode max_speed{14};

    PathLocalizerNode path_localizer{};
    PathLookaheadNode path_lookahead{};
    AwaitSensorNode loop_start_sens{LOOP_START_SID};
    RepeatNode loop_start_wait{&loop_start_sens, 1};
    AwaitSensorNode loop_end_sens{LOOP_END_SID};
    RepeatNode do_loop{&loop_end_sens, 1};
    RepeatNode repeat_loop{&loop_start_sens, 3};

    SaveSensorNode save_sensor_node{};
    PrintSteadyStateSpeed steady_state_speed{};
    RepeatNode print_steady_state{&steady_state_speed, 1};

    SetSpeedNode low_speed{2};
    SaveCheckpointNode save_checkpoint{};
    AwaitSensorNode await_sensor{};
    WaitNode wait_node{7000};
    PrintStoppingDistance print_stop_dist{};
    SetSpeedNode zero_speed1{0};

    SetSpeedNode zero_speed{0};
    InvertNode invert_zero_speed{&zero_speed};

    StoppingDistance(uint16_t speed) : max_speed(speed) {
      // default always
      seq.children.push(&save_sensor);

      // localize if nothing is saved
      seq.children.push(&localizer_tree);

      // run the distance until we get to the end
      run_loop.children.push(&create_loop_start_node);
      run_loop.children.push(&loop_path);
      run_loop.children.push(&max_speed);
      run_loop.children.push(&path_localizer);
      run_loop.children.push(&path_lookahead);
      run_loop.children.push(&loop_start_wait);
      run_loop.children.push(&do_loop);
      run_loop.children.push(&repeat_loop);
      run_loop.children.push(&zero_speed);
      run_loop.children.push(&print_steady_state);
      seq.children.push(&run_loop);

      // now we slowly go forward
      seq.children.push(&wait_node);
      seq.children.push(&save_checkpoint);
      seq.children.push(&low_speed);
      seq.children.push(&await_sensor);
      seq.children.push(&print_stop_dist);
      seq.children.push(&zero_speed1);

      // set to zero speed and print failure
      tree.children.push(&seq);
      tree.children.push(&invert_zero_speed);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
  };

  struct SpeedTester : public TreeNode {
    FallBackNode tree{};

    SequenceNode seq{};

    SaveSensorNode save_sensor{};
    InitalLocalizeTree localizer_tree{};

    PathToNode create_loop_start_node{};
    DebugPrintPath debug_print{};
    RepeatNode debug_print_path{&debug_print, 1};

    AddLoop add_loop{};
    SetSpeedNode max_speed{14};

    PathLocalizerNode path_localizer{};
    PathLookaheadNode path_lookahead{};
    StopAtDonePath stop_on_finish_path{};
    AwaitSensorNode await_sensor_node{sid('E', 6)};
    RepeatNode repeat_node{&await_sensor_node, 2};
    SaveSensorNode save_sensor_node{};
    PrintSteadyStateSpeed steady_state_speed{};

    SetSpeedNode zero_speed{0};
    InvertNode invert_zero_speed{&zero_speed};

    SpeedTester(uint16_t speed) : max_speed(speed) {
      // default always
      seq.children.push(&save_sensor);

      // localize if nothing is saved
      seq.children.push(&localizer_tree);

      // try to run the stop distance thing
      seq.children.push(&create_loop_start_node);
      seq.children.push(&add_loop);
      seq.children.push(&max_speed);
      seq.children.push(&path_localizer);
      // seq.children.push(&path_lookahead);
      seq.children.push(&repeat_node);
      seq.children.push(&zero_speed);
      seq.children.push(&steady_state_speed);

      // set to zero speed and print failure
      tree.children.push(&seq);
      tree.children.push(&invert_zero_speed);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
  };

} // namespace

static void run_tree(TreeNode &tree, Blackboard &bb) {
  while (true) {
    auto next_msg = send<TC::TreeMsg>(bb.tcs_tid, TC::TreeReady{});
    if (!next_msg.has_value()) {
      break;
    }

    std::visit(
        [&](auto &&event) {
          using Event = std::decay_t<decltype(event)>;

          if constexpr (std::is_same_v<Event, TC::InitTree>) {
            bb.state        = event.state;
            bb.loco_id      = event.loco_id;
            bb.target_speed = event.value;
          } else if constexpr (std::is_same_v<Event, TC::TreeUpdate>) {
            bb.state.update_from_mrk(event.mrk);
            bb.new_event  = event.mrk;
            bb.event_tick = event.time;
          }
        },
        next_msg.value());
    auto result = tree.tick(bb);
    if (result == NodeResult::Failure) {
      Debug_Puts(bb.txs_tid, "Error: ", bb.error_msg);
      break;
    } else if (result == NodeResult::Success) {
      break;
    }
  }
}

// void train_tree_task() {
//   auto tcs_tid     = WhoIs(TrainControlServer<>::NAME);
//   auto tx_tid      = WhoIs(UART_TX_Server::NAME);
//   auto cs_tid      = WhoIs(ClockServer<>::NAME);
//   uint32_t loco_id = 1;
//   State state{};

//   for (uint16_t i = 14; i > 0; --i) {
//     Debug_Puts(tx_tid, "Running tree with speed ", i, "\n\r");
//     SpeedTester tree{i};
//     Blackboard bb{
//         .tcs_tid = tcs_tid,
//         .txs_tid = tx_tid,
//         .cs_tid  = cs_tid,
//         .pathfinder{'a'},
//         .state   = state,
//         .loco_id = loco_id,
//     };
//     run_tree(tree, bb);
//     loco_id = bb.loco_id;
//     state   = bb.state;
//   }
//   std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
// }

void train_tree_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);
  auto cs_tid  = WhoIs(ClockServer<>::NAME);

  StoppingDistance tree{10};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .cs_tid  = cs_tid,
      .pathfinder{'a'},
  };
  run_tree(tree, bb);
  std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
}