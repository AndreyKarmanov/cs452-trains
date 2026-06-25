
#include "train_server.h"
#include "behaviour_tree.h"
#include "io_helpers.h"
#include "message.h"
#include "mrk.h"
#include "pathfind.h"
#include "time.h"
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

  struct CalculateSteadySpeed : public LeafNode {
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

      auto total_ticks                     = last_tick - first_tick;
      auto estimated_speed                 = (total_dist * 1000) / total_ticks;
      auto loco                            = bb.state.get_loco(bb.loco_id);
      loco.top_speed[loco.requested_speed] = estimated_speed;

      Debug_Puts(bb.txs_tid, "Speed ",
                 bb.state.get_loco(bb.loco_id).requested_speed,
                 " Total dist: ", total_dist, "mm, total ticks: ", total_ticks,
                 " speed: ", estimated_speed,
                 "tmm/tick Sensors used: ", measurements_used, "\n\r");

      return NodeResult::Success;
    }
  };

  struct CalculateAccel : public LeafNode {
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

      auto T    = last_tick - first_tick;
      auto loco = bb.state.get_loco(bb.loco_id);

      auto vf    = loco.top_speed[loco.requested_speed];
      loco.accel = (vf * vf * 1000) / (2 * (vf * T - total_dist * 1000));

      Debug_Puts(bb.txs_tid, "Speed ",
                 bb.state.get_loco(bb.loco_id).requested_speed,
                 " Total dist: ", total_dist, "mm, total ticks: ", T,
                 " acceleration: ", loco.accel,
                 "um/ktick Sensors used: ", measurements_used, "\n\r");

      return NodeResult::Success;
    }
  };

  struct PrintStoppingDistance : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      auto loco = bb.state.get_loco(bb.loco_id);
      auto b    = loco.top_speed[loco.requested_speed];
      // auto time_to_accel = (b * 1000) / loco.accel; // in ticks
      auto measured_dist_tmm  = (bb.event_tick - bb.last_checkpoint) * b;
      auto stopped_sensor_cmd = LOOP_START_SID;

      uint16_t total_dist = 0;
      for (auto it = bb.dists.end() - 1; it != bb.dists.begin(); --it) {
        if ((*it).sensor_data.sensor_id == stopped_sensor_cmd) {
          break;
        }
        total_dist += (*it).distance;
      }
      auto stopping = (total_dist * 1000 - measured_dist_tmm) / 1000;

      Debug_Puts(bb.txs_tid, "Measured: ", measured_dist_tmm,
                 "um, Stopping: ", stopping, "mm b", b, " last checkpoint ",
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

  struct TrackStop : public LeafNode {
    bool setStop;
    NodeResult tick(Blackboard &bb) override {

      if (setStop || setStop == bb.state.stopped) {
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Stop{});
      if (!resp.has_value()) {
        return NodeResult::Failure;
      }
      setStop = true;
      return NodeResult::Success;
    }
  };

  struct TrackGo : public LeafNode {
    bool setStop;
    NodeResult tick(Blackboard &bb) override {

      if (setStop || setStop == bb.state.stopped) {
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Go{});
      if (!resp.has_value()) {
        return NodeResult::Failure;
      }
      setStop = true;
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

  struct SensorPredict : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      auto loco = bb.state.get_loco(bb.loco_id);

      auto delta_tick_cmd = bb.event_tick - loco.req_spd_tick;
      uint16_t accel_spd  = (delta_tick_cmd * loco.accel) / 1000;
      bb.est_speed = std::min(loco.top_speed[loco.requested_speed], accel_spd);

      auto delta_tick_evnt    = bb.event_tick - bb.last_tick;
      bb.dist_to_next_sensor -= (bb.est_speed * delta_tick_evnt) / 1000;
      bb.lookahead           += (bb.est_speed * delta_tick_evnt) / 1000;
      bb.last_tick            = bb.event_tick;

      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1) {
        Debug_Puts(bb.txs_tid, "Sensor Delta ", bb.dist_to_next_sensor,
                   " Est Speed ", bb.est_speed, "um/ms Lookahead ",
                   bb.lookahead, "mm\n\r");
        bb.lookahead = (bb.est_speed * TICKS_PER_S * 3) / 1000;

        auto next_sensor_idx = std::ranges::find_if(
            bb.path, [](PathNode &node) { return node.type == NODE_SENSOR; });
        if (next_sensor_idx == bb.path.end()) {
          return NodeResult::Success;
        }

        bb.dist_to_next_sensor =
            std::accumulate(bb.path.begin(), next_sensor_idx + 1, uint16_t(0),
                            [](uint16_t acc, const PathNode &node) {
                              return acc + node.distance_to_prev_node;
                            });
      }
      return NodeResult::Success;
    }
  };

  struct PathLookaheadNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        return NodeResult::Success;
      }

      // calculate distance travelled given current velocity
      // safe estimate is max velocity for speed
      // then, calculate distance based on velocity. suppose distance is 500
      uint32_t total_dist = 0;
      for (auto &node : bb.path) {
        total_dist += node.distance_to_prev_node;
        if (total_dist > bb.lookahead) {
          break;
        }

        if (node.type == NODE_BRANCH) {
          if (node.should_br_be_curved &&
              bb.state.is_switch_straight(node.num)) {
            Debug_Puts(bb.txs_tid, "Setting switch ", node.num,
                       " to curved\n\r");
            auto res =
                send<TC::Ack>(bb.tcs_tid, TC::Cmd::Switch(node.num, false));
            if (!res.has_value()) {
              return NodeResult::Failure;
            }
          } else if (!node.should_br_be_curved &&
                     !bb.state.is_switch_straight(node.num)) {
            Debug_Puts(bb.txs_tid, "Setting switch ", node.num,
                       " to straight\n\r");
            auto res =
                send<TC::Ack>(bb.tcs_tid, TC::Cmd::Switch(node.num, true));
            if (!res.has_value()) {
              return NodeResult::Failure;
            }
          }
        }
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
    const char *goal;
    bool path_initalized = false;

    PathToNode(const char *goal = nullptr) : goal(goal) {}

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

      const char *goal_name = goal != nullptr ? goal : bb.nav_goal.c_str();
      auto goal_idx         = bb.pathfinder.get_idx(goal_name);
      if (!goal_idx.has_value()) {
        bb.error_msg = "Failed to find goal";
        return NodeResult::Failure;
      }

      auto path_opt = bb.pathfinder.shortest_path(start_idx, goal_idx.value());

      if (!path_opt.has_value()) {
        Debug_Puts(bb.txs_tid, "Failed to find path from ", start_idx, " to ",
                   goal_idx.value(), "\n\r");
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
        bb.error_msg = "Failed to find loop";
        return NodeResult::Failure;
      }
      bb.path = bb.path + path_opt.value();
      return NodeResult::Success;
    }
  };

  struct CalibrateTrain : public LeafNode {
    FallBackNode tree{};

    SequenceNode seq{};
    SequenceNode setup_loop{};
    SequenceNode measure_top_speed{};
    SequenceNode measure_acc_to_top{};
    SequenceNode measure_stopping{};

    SaveSensorNode save_sensor{};
    InitalLocalizeTree localizer_tree{};

    PathToNode create_loop_start_node{"B6"};
    DebugPrintPath debug_print{};
    RepeatNode debug_print_path{&debug_print, 1};

    AddLoop loop_path{};
    SetSpeedNode localize_speed{5};

    SetTargetSpeedNode max_speed{14};
    SetTargetSpeedNode max_speed2{14};

    PathLocalizerNode path_localizer{};
    SensorPredict sensor_predict{};
    PathLookaheadNode path_lookahead{};
    AwaitSensorNode loop_start_sens{LOOP_START_SID};
    RepeatNode loop_start_wait{&loop_start_sens, 1};
    RepeatNode repeat_loop{&loop_start_sens, 4};
    RepeatNode repeat_loop2{&loop_start_sens, 2};
    RepeatNode repeat_loop3{&loop_start_sens, 2};

    SaveSensorNode save_sensor_node{};
    CalculateSteadySpeed steady_state_speed{};
    CalculateAccel calculate_accel{};
    SetSpeedNode slow_speed{7};
    RepeatNode print_top_speed{&steady_state_speed, 1};
    RepeatNode print_accel{&calculate_accel, 1};

    SetSpeedNode low_speed{2};
    SaveCheckpointNode save_checkpoint{};
    AwaitSensorNode await_sensor{};
    WaitNode wait_node{2000};
    WaitNode wait_node1{7000};
    PrintStoppingDistance print_stop_dist{};

    TrackStop track_stop{};
    TrackGo track_go{};
    SetSpeedNode zero_speed{0};
    SetSpeedNode zero_speed1{0};
    SetSpeedNode zero_speed2{0};

    InvertNode invert_zero_speed{&zero_speed2};

    CalibrateTrain(uint16_t speed) : max_speed(speed) {
      // default always
      seq.children.push(&save_sensor);

      // localize if nothing is saved
      seq.children.push(&localizer_tree);

      setup_loop.children.push(&create_loop_start_node);
      setup_loop.children.push(&localize_speed);
      setup_loop.children.push(&path_localizer);
      setup_loop.children.push(&sensor_predict);
      setup_loop.children.push(&path_lookahead);
      setup_loop.children.push(&loop_start_wait);
      setup_loop.children.push(&loop_path);
      seq.children.push(&setup_loop);

      // we enter this at top speed, loop 3 times, and measure time at top
      // speed
      measure_top_speed.children.push(&max_speed);
      measure_top_speed.children.push(&repeat_loop);
      measure_top_speed.children.push(&print_top_speed);
      seq.children.push(&measure_top_speed);

      // insta-stop train at the start of loop (wait 2 s for stop)
      // then do the loop twice, getting the time to loop
      measure_acc_to_top.children.push(&slow_speed);
      measure_acc_to_top.children.push(&repeat_loop2);
      measure_acc_to_top.children.push(&track_stop);
      measure_acc_to_top.children.push(&wait_node);
      measure_acc_to_top.children.push(&track_go);
      measure_acc_to_top.children.push(&max_speed2);
      measure_acc_to_top.children.push(&repeat_loop3);
      measure_acc_to_top.children.push(&print_accel);
      seq.children.push(&measure_acc_to_top);

      // now we slowly go forward
      measure_stopping.children.push(&zero_speed);
      measure_stopping.children.push(&wait_node1);
      measure_stopping.children.push(&save_checkpoint);
      measure_stopping.children.push(&low_speed);
      measure_stopping.children.push(&await_sensor);
      measure_stopping.children.push(&print_stop_dist);
      seq.children.push(&measure_stopping);

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

    PathToNode create_loop_start_node{"B6"};
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
    CalculateSteadySpeed steady_state_speed{};

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

  struct NavigateTree : public TreeNode {
    SequenceNode seq{};

    InitalLocalizeTree localizer_tree{};
    SaveSensorNode save_sensor{};
    PathToNode path_to_goal{};
    PathLocalizerNode path_localizer{};
    PathLookaheadNode path_lookahead{};
    SetTargetSpeedNode max_speed{7};
    StopAtDonePath stop_at_done{};

    NavigateTree() {
      seq.children.push(&save_sensor);
      seq.children.push(&localizer_tree);
      seq.children.push(&path_to_goal);
      seq.children.push(&max_speed);
      seq.children.push(&path_localizer);
      seq.children.push(&path_lookahead);
      seq.children.push(&stop_at_done);
    }

    NodeResult tick(Blackboard &bb) override {
      if (!bb.error_msg.empty()) {
        return NodeResult::Failure;
      }
      return seq.tick(bb);
    }
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
          } else if constexpr (std::is_same_v<Event, TC::InitNav>) {
            bb.state        = event.state;
            bb.loco_id      = event.loco_id;
            bb.target_speed = event.speed;
            bb.nav_goal     = event.to;

            if (!bb.pathfinder.get_idx(event.to.c_str()).has_value()) {
              bb.error_msg = "Unknown to node";
            }
          } else if constexpr (std::is_same_v<Event, TC::TreeUpdate>) {
            bb.state.update_from_mrk(event.mrk, event.time);
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

  CalibrateTrain tree{10};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .cs_tid  = cs_tid,
      .pathfinder{'b'},
  };
  run_tree(tree, bb);
  std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
}

void nav_tree_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);
  auto cs_tid  = WhoIs(ClockServer<>::NAME);

  NavigateTree tree{};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .cs_tid  = cs_tid,
      .pathfinder{'b'},
  };
  run_tree(tree, bb);
  std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
}
