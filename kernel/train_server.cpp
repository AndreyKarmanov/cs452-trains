
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
  constexpr int LOOP_START_SID = sid('D', 4);
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

      uint32_t last_tick   = 0;
      uint32_t first_tick  = 0;
      uint32_t ttl_dist_um = 0;

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
          ttl_dist_um += (*it).distance * 1000;
          measurements_used++;
        }
      }

      auto total_ticks                   = last_tick - first_tick;
      auto estimated_speed               = ttl_dist_um / total_ticks;
      bb.loco->v_max[bb.loco->req_speed] = estimated_speed;
      bb.top_loop_time                   = total_ticks;

      Debug_Puts(bb.txs_tid, "Speed ", bb.loco->req_speed,
                 " Total dist: ", ttl_dist_um / 1000,
                 "mm, total ticks: ", total_ticks, " speed: ", estimated_speed,
                 "tmm/tick Sensors used: ", measurements_used, "");

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

      uint32_t last_tick   = 0;
      uint32_t first_tick  = 0;
      uint32_t ttl_dist_um = 0;

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
          ttl_dist_um += (*it).distance * 1000;
          measurements_used++;
        }
      }

      auto T  = last_tick - first_tick;
      auto vf = bb.loco->v_max[bb.loco->req_speed];
      bb.loco->accel[bb.loco->req_speed] =
          (vf * vf * 1000) / (2 * (vf * T - ttl_dist_um));
      bb.accel_loop_time = T;

      Debug_Puts(bb.txs_tid, "Speed ", bb.loco->req_speed,
                 " Total dist: ", ttl_dist_um / 1000, "mm, total ticks: ", T,
                 " acceleration: ", bb.loco->accel[bb.loco->req_speed],
                 "um/ktick Sensors used: ", measurements_used, "");

      return NodeResult::Success;
    }
  };

  struct PrintStoppingDistance : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      uint32_t last_tick  = bb.curr_tick;
      uint32_t first_tick = bb.saved_tick;

      auto total_ticks  = last_tick - first_tick;
      auto top_speed    = bb.loco->v_max[bb.loco->req_speed];
      auto dist_to_stop = (bb.accel_loop_time - total_ticks) / top_speed;

      Debug_Puts(bb.txs_tid, "Speed ", bb.loco->req_speed,
                 " total ticks: ", total_ticks, " stop dist: ", dist_to_stop);

      return NodeResult::Success;
    }
  };

  struct SaveCheckpointNode : public LeafNode {
    bool saved = false;
    NodeResult tick(Blackboard &bb) override {
      if (!saved) {
        bb.saved_tick = bb.curr_tick;
        saved         = true;
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

      if (set_speed || req_speed == bb.loco->req_speed) {
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

      if (set_speed || req_speed == bb.loco->req_speed) {
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

      if (backward == bb.loco->backward) {
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
            .tick = bb.curr_tick,
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
          Debug_Puts(bb.txs_tid, "Couldn't find self in path");
          return NodeResult::Failure;
        }

        if (bb.dists.size() == bb.dists.capacity()) {
          bb.dists.pop();
        }
        bb.dists.push({
            .distance = std::accumulate(bb.path.begin(), idx + 1, uint16_t(0),
                                        [](uint16_t acc, const PathNode &node) {
                                          return acc + node.dx_prev;
                                        }),
            .tick     = bb.curr_tick,
            .sensor_data = *data,
        });
        auto skipped_nodes = std::distance(bb.path.begin(), idx) + 1;
        bb.path.pop(skipped_nodes);

        // StaticString<128> path_str{};
        // path_str.append("Path: ", bb.path.size(), " ");
        // for (auto node : bb.path) {
        //   path_str.append(bb.pathfinder.track[node.node_idx].name, " ",
        //                   node.dx_next, " >");
        // }
        // Debug_Puts(bb.txs_tid, path_str);
      }
      return NodeResult::Success;
    }
  };

  struct UpdateModel : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      uint32_t a     = bb.loco->accel[bb.loco->req_speed];
      uint32_t v_max = bb.loco->v_max[bb.loco->req_speed];
      uint32_t v_i   = bb.loco->ve;
      uint32_t d_t   = bb.curr_tick - bb.last_tick;

      uint32_t t_a = std::min(((v_max - v_i) * 1000 / a), d_t);
      uint32_t t_c = d_t - t_a;

      uint32_t dx_um = ((a * t_a * t_a) / 2000 + v_max * t_c + v_i * d_t);

      bb.dx_next_sens_um -= dx_um;
      bb.lookahead_um    += dx_um;
      bb.last_tick        = bb.curr_tick;
      uint32_t v_f        = (v_i * 1000 + a * t_a) / 1000;
      bb.loco->ve         = std::min(v_max, v_f);

      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1) {

        Debug_Puts(bb.txs_tid, "Sensor Delta ", bb.dx_next_sens_um / 1000,
                   "mm Est Speed ", bb.loco->ve, "um/ms Lookahead ",
                   bb.lookahead_um / 1000, "mm ", t_a, " ta");

        bb.lookahead_um =
            std::max(bb.loco->ve * TICKS_PER_S * 2, 1500u * 1000u);

        auto next_sensor_idx = std::ranges::find_if(
            bb.path, [](PathNode &node) { return node.type == NODE_SENSOR; });
        if (next_sensor_idx == bb.path.end()) {
          return NodeResult::Success;
        }

        bb.dx_next_sens_um =
            1000 * std::accumulate(bb.path.begin(), next_sensor_idx + 1,
                                   uint16_t(0),
                                   [](uint16_t acc, const PathNode &node) {
                                     return acc + node.dx_prev;
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
        total_dist += node.dx_prev;
        if (total_dist > (bb.lookahead_um / 1000)) {
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
          } else if (!node.should_br_be_curved &&
                     !bb.state.is_switch_straight(node.num)) {
            // Debug_Puts(bb.txs_tid, "Setting switch ", node.num, " to
            // straight");
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
                   goal_idx.value());
        return NodeResult::Failure;
      }
      bb.path         = path_opt.value();
      path_initalized = true;
      return NodeResult::Success;
    }
  };

  struct AddLoop : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      if (bb.path.empty() && bb.seen_sensors.empty()) {
        bb.error_msg = "No path to loop";
        return NodeResult::Failure;
      }

      if (bb.path.empty()) {
        auto sens = bb.seen_sensors.peek_last();
        bb.path.push({
            .node_idx            = sens->sid - 1,
            .type                = NODE_SENSOR,
            .num                 = sens->sid,
            .dx_prev             = 0,
            .dx_next             = 0,
            .should_br_be_curved = false,
        });
      }

      if (bb.path.size() > 1 &&
          bb.path.peek()->node_idx == bb.path.peek_last()->node_idx) {
        return NodeResult::Success;
      }

      auto path_opt = bb.pathfinder.shortest_path(bb.path.peek_last()->node_idx,
                                                  bb.path.peek()->node_idx);

      if (!path_opt.has_value()) {
        bb.error_msg = "Failed to find loop";
        return NodeResult::Failure;
      }
      auto new_path = path_opt.value();
      if (bb.path.peek()->type == NODE_BRANCH) {
        auto &last_node               = *(new_path.end() - 1);
        auto &first_node              = *(bb.path.begin());
        last_node.should_br_be_curved = first_node.should_br_be_curved;
      }
      bb.path = bb.path + new_path;
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

    SetSpeedNode localize_speed{7};
    SetTargetSpeedNode max_speed{14};
    SetTargetSpeedNode max_speed2{14};
    SetTargetSpeedNode max_speed3{14};

    PathLocalizerNode path_localizer{};
    UpdateModel sensor_predict{};
    PathLookaheadNode path_lookahead{};
    AwaitSensorNode loop_start_sens{LOOP_START_SID};
    RepeatNode loop_start_wait{&loop_start_sens, 1};
    RepeatNode repeat_loop{&loop_start_sens, 4};
    RepeatNode repeat_loop2{&loop_start_sens, 2};
    RepeatNode repeat_loop3{&loop_start_sens, 2};
    RepeatNode repeat_loop4{&loop_start_sens, 2};

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
    PrintStoppingDistance print_stopping{};

    TrackStop track_stop{};
    TrackGo track_go{};
    SetSpeedNode zero_speed{0};
    SetSpeedNode zero_speed1{0};
    SetSpeedNode zero_speed2{0};

    InvertNode invert_zero_speed{&zero_speed2};

    CalibrateTrain(uint16_t speed) : max_speed(speed), max_speed2(speed) {
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
      measure_stopping.children.push(&max_speed3);
      measure_stopping.children.push(&repeat_loop4);
      measure_stopping.children.push(&print_stopping);
      seq.children.push(&measure_stopping);

      seq.children.push(&zero_speed1);

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
  TrainState tmp{};
  bb.loco = &tmp;
  while (true) {
    auto next_msg =
        send<TC::TreeMsg>(bb.tcs_tid, TC::TreeReady{.train = *bb.loco});
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
            bb.loco         = bb.state.get_loco(bb.loco_id);
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
            bb.new_event = event.mrk;
            bb.curr_tick = event.time;
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
//     Debug_Puts(tx_tid, "Running tree with speed ", i, "");
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
      .pathfinder{'a'},
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
