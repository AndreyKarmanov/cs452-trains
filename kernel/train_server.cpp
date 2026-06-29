
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
  constexpr auto TRACK           = 'b';
  constexpr auto LOOP_START_NODE = "D4";
  constexpr int LOOP_START_SID   = sid('D', 4);
  constexpr size_t CRAWL_SPEED   = 4;

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

  struct PrintTrainStats : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      // print all the acceleration values and top speed values for the train
      Debug_Puts(bb.txs_tid, "Train ", bb.loco_id, " Stats: ");
      Debug_Puts(bb.txs_tid, "Speed | Top | Accel | Stop Dist");
      for (int i = 0; i < 15; ++i) {
        StaticString<128> stats_str{};
        AppendPadded(stats_str, i, 3);
        stats_str.append("   | ");
        AppendPadded(stats_str, bb.loco->v_max[i], 3);
        stats_str.append(" | ");
        AppendPadded(stats_str, bb.loco->accel[i], 5);
        stats_str.append(" | ", bb.loco->stop_dist_um[i] / 1000);
        Debug_Puts(bb.txs_tid, stats_str, "\n\r");
      }

      return NodeResult::Success;
    }
  };

  struct CalculateSteadySpeed : public LeafNode {
    bool done{false};
    NodeResult tick(Blackboard &bb) override {
      if (done) {
        return NodeResult::Success;
      }
      bool found_start      = false;
      int loops_to_use      = 2;
      int measurements_used = 0;

      uint32_t ttl_dist_um = 0;
      uint32_t ttl_ticks   = 0;

      for (auto it = bb.dists.end() - 1;
           it != bb.dists.begin() && loops_to_use > 0; it--) {
        if ((*it).sensor_data.sensor_id == LOOP_START_SID) {
          if (found_start == false) {
            found_start = true;
          } else {
            loops_to_use--;
          }
        }
        if (found_start && loops_to_use > 0) {
          ttl_dist_um += (*it).dx_um;
          ttl_ticks   += (*it).d_ticks;
          measurements_used++;
          Debug_Puts(bb.txs_tid, "f: ", (*it).from_sid, " to: ", (*it).to_sid,
                     " d: ", (*it).dx_um / 1000, "mm in: ", (*it).d_ticks);
        }
      }

      auto estimated_speed            = ttl_dist_um / ttl_ticks;
      bb.loco->v_max[bb.target_speed] = estimated_speed;
      bb.top_loop_time                = ttl_ticks;

      Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
                 " Total dist: ", ttl_dist_um / 1000,
                 "mm, total ticks: ", ttl_ticks, " speed: ", estimated_speed,
                 "um/tick Sensors used: ", measurements_used, "");

      done = true;
      return NodeResult::Success;
    }
  };

  struct CalculateAccel : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      bool found_start      = false;
      int loops_to_use      = 1;
      int measurements_used = 0;

      uint32_t ttl_dist_um = 0;
      uint32_t ttl_ticks   = 0;

      for (auto it = bb.dists.end() - 1;
           it != bb.dists.begin() && loops_to_use > 0; it--) {
        if ((*it).sensor_data.sensor_id == LOOP_START_SID) {
          if (found_start == false) {
            found_start = true;
          } else {
            loops_to_use--;
          }
        }
        if (found_start && loops_to_use > 0) {
          ttl_dist_um += (*it).dx_um;
          ttl_ticks   += (*it).d_ticks;
          measurements_used++;

          Debug_Puts(bb.txs_tid, "f: ", (*it).from_sid, " to: ", (*it).to_sid,
                     " d: ", (*it).dx_um / 1000, "mm in: ", (*it).d_ticks);
        }
      }

      auto vc = bb.loco->v_max[CRAWL_SPEED];
      auto vf = bb.loco->v_max[bb.target_speed];

      auto accel = ((vf * vf - 2 * vf * vc + vc * vc) * 1000) /
                   (2 * (vf * ttl_ticks - ttl_dist_um));

      bb.loco->accel[bb.target_speed] = accel;

      Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
                 " Total dist: ", ttl_dist_um / 1000,
                 "mm, total ticks: ", ttl_ticks,
                 " acceleration: ", bb.loco->accel[bb.target_speed],
                 "nm/tick^2 sensors: ", measurements_used);

      return NodeResult::Success;
    }
  };

  struct CalculateStop : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      bool found_start      = false;
      int loops_to_use      = 1;
      int measurements_used = 0;

      uint32_t ttl_dist_um = 0;
      uint32_t ttl_ticks   = 0;

      for (auto it = bb.dists.end() - 1;
           it != bb.dists.begin() && loops_to_use > 0; it--) {
        if ((*it).sensor_data.sensor_id == LOOP_START_SID) {
          if (found_start == false) {
            found_start = true;
          } else {
            loops_to_use--;
          }
        }
        if (found_start && loops_to_use > 0) {
          ttl_dist_um += (*it).dx_um;
          ttl_ticks   += (*it).d_ticks;
          measurements_used++;

          Debug_Puts(bb.txs_tid, "f: ", (*it).from_sid, "to: ", (*it).to_sid,
                     " d: ", (*it).dx_um / 1000, "mm in: ", (*it).d_ticks);
        }
      }

      auto vc = bb.loco->v_max[CRAWL_SPEED];
      auto vf = bb.loco->v_max[bb.target_speed];

      auto decel = ((vf * vf - 2 * vf * vc + vc * vc) * 1000) /
                   (2 * (ttl_dist_um - vc * ttl_ticks));

      auto stop_dist                         = ttl_dist_um - vc * ttl_ticks;
      bb.loco->decel[bb.target_speed]        = decel;
      bb.loco->stop_dist_um[bb.target_speed] = stop_dist;

      Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
                 " Total dist: ", ttl_dist_um / 1000,
                 "mm, total ticks: ", ttl_ticks, " deceleration: ", decel,
                 "nm/tick^2 stop dist: ", stop_dist / 1000,
                 "mm sensors: ", measurements_used);

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

  struct SetSpeed : public LeafNode {
    uint16_t req_speed;
    bool reached_speed = false;
    bool sent_cmd      = false;
    SetSpeed(uint16_t speed) : req_speed(speed) {}
    NodeResult tick(Blackboard &bb) override {

      // if sent + reached -> done
      if (reached_speed) {
        return NodeResult::Success;
      }

      // if sent + got response -> done
      if (auto data = std::get_if<SpeedCmd>(&bb.new_event);
          sent_cmd && data && data->loco_id == bb.loco_id &&
          mrk_level_to_user_speed(data->speed) == req_speed) {
        reached_speed = true;
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(
          bb.tcs_tid, TC::Cmd::Speed{.id = bb.loco_id, .value = req_speed});
      if (!resp.has_value()) {
        bb.error_msg = "Failed to set speed";
        return NodeResult::Failure;
      }
      sent_cmd = true;
      return NodeResult::Running;
    }
  };

  struct SetTargetSpeed : public LeafNode {
    uint16_t req_speed;
    bool reached_speed = false;
    bool sent_cmd      = false;
    SetTargetSpeed(uint16_t speed) : req_speed(speed) {}
    NodeResult tick(Blackboard &bb) override {

      // if sent + reached -> done
      if (reached_speed) {
        return NodeResult::Success;
      }

      // if sent + got response -> done
      if (auto data = std::get_if<SpeedCmd>(&bb.new_event);
          sent_cmd && data && data->loco_id == bb.loco_id &&
          mrk_level_to_user_speed(data->speed) == bb.target_speed) {
        reached_speed = true;
        return NodeResult::Success;
      }

      auto resp =
          send<TC::Ack>(bb.tcs_tid, TC::Cmd::Speed{.id    = bb.loco_id,
                                                   .value = bb.target_speed});
      if (!resp.has_value()) {
        bb.error_msg = "Failed to set speed";
        return NodeResult::Failure;
      }
      sent_cmd = true;
      return NodeResult::Running;
    }
  };

  struct TrackStop : public LeafNode {
    bool set = false;
    NodeResult tick(Blackboard &bb) override {
      if (set || bb.state.stopped) {
        set = true;
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Stop{});
      if (!resp.has_value()) {
        bb.error_msg = "Failed to stop track";
        return NodeResult::Failure;
      }
      return NodeResult::Running;
    }
  };

  struct TrackGo : public LeafNode {
    bool set = false;
    NodeResult tick(Blackboard &bb) override {
      if (set || !bb.state.stopped) {
        set = true;
        return NodeResult::Success;
      }

      auto resp = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Go{});
      if (!resp.has_value()) {
        bb.error_msg = "Failed to go track";
        return NodeResult::Failure;
      }
      return NodeResult::Running;
    }
  };

  struct SetDirectionNode : public LeafNode {
    bool backward;
    SetDirectionNode(bool backward) : backward(backward) {}
    NodeResult tick(Blackboard &bb) override {
      if (backward == bb.loco->backward) {
        return NodeResult::Success;
      }

      auto resp =
          send<TC::Ack>(bb.tcs_tid, TC::Cmd::Direction{.id       = bb.loco_id,
                                                       .backward = backward});
      if (!resp.has_value()) {
        bb.error_msg = "Failed to set direction";
        return NodeResult::Failure;
      }
      return NodeResult::Running;
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

  struct LocalizerNode : public LeafNode {
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

        if (bb.path.empty()) {
          return NodeResult::Success;
        }

        auto sensor_id_cmp = [&](PathNode &node) {
          if (node.type == NODE_SENSOR) {
            return node.node_idx + 1;
          }
          return -1;
        };

        auto idx = std::ranges::find(bb.path, data->sensor_id, sensor_id_cmp);
        if (idx == bb.path.end()) {
          Debug_Puts(bb.txs_tid, "Couldn't find ", data->sensor_id,
                     (char)('A' + data->bank), data->number, " in path");
          StaticString<128> path_str{};
          path_str.append("Path: ", bb.path.size(), " ");
          for (auto node : bb.path) {
            path_str.append(bb.pathfinder.track[node.node_idx].name, " ",
                            node.dx_next, " >");
          }
          Debug_Puts(bb.txs_tid, path_str);
          return NodeResult::Failure;
        }

        uint32_t dx_mm =
            std::accumulate(bb.path.begin(), idx + 1, uint32_t(0),
                            [](uint32_t acc, const PathNode &node) {
                              return acc + static_cast<uint32_t>(node.dx_prev);
                            });

        if (bb.last_sensor_sid != 0 && dx_mm > 0) {
          if (bb.dists.size() == bb.dists.capacity()) {
            bb.dists.pop();
          }
          bb.dists.push({
              .from_sid    = bb.last_sensor_sid,
              .to_sid      = data->sensor_id,
              .dx_um       = dx_mm * 1000,
              .d_ticks     = bb.curr_tick - bb.last_sensor_ticks,
              .sensor_data = *data,
          });
        }
        bb.last_sensor_sid   = data->sensor_id;
        bb.last_sensor_ticks = bb.curr_tick;

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
      int a     = bb.loco->accel[bb.loco->req_speed];
      int v_max = bb.loco->v_max[bb.loco->req_speed];
      int v_i   = bb.loco->ve;
      int d_t   = bb.curr_tick - bb.last_tick;

      int t_a = std::min(((v_max - v_i) * 1000 / a), d_t);
      int t_c = d_t - t_a;

      int dx_um = ((a * t_a * t_a) / 2000 + v_max * t_c + v_i * d_t);

      bb.dx_um     += static_cast<uint32_t>(dx_um);
      bb.last_tick  = bb.curr_tick;
      bb.loco->ve   = (v_i * 1000 + a * t_a) / 1000;

      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1) {

        auto one_ago = *(bb.dists.end() - 1);
        bb.loco->ve  = (3 * bb.loco->ve + one_ago.dx_um / one_ago.d_ticks) / 4;

        int dist_prev = bb.dists.empty() ? 0 : bb.dists.peek_last()->dx_um;
        Offset_Puts(bb.txs_tid, -1, "Spd: ", bb.loco->ve, "um/ms ",
                    (static_cast<int>(bb.dx_um) - dist_prev) / 1000,
                    "mm Error");

        bb.dx_um = 0;

        auto next_sensor_idx = std::ranges::find_if(
            bb.path, [](PathNode &node) { return node.type == NODE_SENSOR; });
        if (next_sensor_idx == bb.path.end()) {
          return NodeResult::Success;
        }
      }
      return NodeResult::Success;
    }
  };

  struct PathLookaheadNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        return NodeResult::Success;
      }

      auto lookahead_um =
          bb.dx_um + static_cast<uint32_t>(bb.loco->ve) * TICKS_PER_S * 3;
      bb.lookahead_um =
          lookahead_um > 1500u * 1000u ? lookahead_um : 1500u * 1000u;

      // calculate distance travelled given current velocity
      // safe estimate is max velocity for speed
      // then, calculate distance based on velocity. suppose distance is 500
      uint32_t total_dist = 0;
      for (auto &node : bb.path) {
        total_dist += node.dx_prev;
        if (total_dist * 1000 > (bb.lookahead_um)) {
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
    bool stop_sent{false};

    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        if (!stop_sent) {
          auto res = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Speed(bb.loco_id, 0));
          if (!res.has_value()) {
            return NodeResult::Failure;
          }
          stop_sent = true;
        }
        return NodeResult::Success;
      }

      // uint32_t remaining_um = 0;
      // if (bb.path.dist * 1000u > bb.dx_um) {
      //   remaining_um = static_cast<uint32_t>(bb.path.dist) * 1000u - bb.dx_um
      //   -
      //                  bb.loco->ve * TICKS_PER_MS * 10;
      // }
      // auto stop_dist_um = bb.loco->stop_dist_um[bb.loco->req_speed];

      // Offset_Puts(bb.txs_tid, 1, "Spd: ", bb.loco->ve, "um/ms ",
      //             stop_dist_um / 1000, "mm stop dist ", remaining_um / 1000,
      //             "mm left");

      // if (!stop_sent && remaining_um <= stop_dist_um) {
      //   auto res = send<TC::Ack>(bb.tcs_tid, TC::Cmd::Speed(bb.loco_id, 0));
      //   if (!res.has_value()) {
      //     return NodeResult::Failure;
      //   }
      //   stop_sent = true;
      // }

      return NodeResult::Running;
    };
  };

  // sets the train to move slowly (speed 4) and waits for the first sensor
  // without stopping the train once localization has started.
  struct LocalizerTree : public LeafNode {
    Sequence tree{};

    Sequence inital{};
    SetSpeed set_speed{CRAWL_SPEED};
    AwaitSensorNode await_sensor{};
    Repeat localize_init{&inital, 1};

    Sequence loop{};
    LocalizerNode localize{};
    UpdateModel model{};
    PathLookaheadNode lookahead{};

    LocalizerTree() {
      inital.children.push(&set_speed);
      inital.children.push(&await_sensor);
      tree.children.push(&localize_init);

      loop.children.push(&localize);
      loop.children.push(&model);
      loop.children.push(&lookahead);
      tree.children.push(&loop);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
  };

  struct PathToNode : public LeafNode {
    const char *goal;
    PathToNode(const char *goal = nullptr) : goal(goal) {}

    NodeResult tick(Blackboard &bb) override {
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
      auto last_node = bb.path.peek_last();
      if (last_node.has_value() && last_node->node_idx == goal_idx.value()) {
        // already on a path to the goal.
        return NodeResult::Success;
      }

      auto path_opt = bb.pathfinder.shortest_path(start_idx, goal_idx.value());

      if (!path_opt.has_value()) {
        Debug_Puts(bb.txs_tid, "Failed to find path from ", start_idx, " to ",
                   goal_idx.value());
        return NodeResult::Failure;
      }
      bb.path = path_opt.value();
      return NodeResult::Success;
    }
  };

  struct CalibrateTrain : public LeafNode {

    Sequence test_seq{};
    LocalizerTree localize_tree{};

    Sequence setup_loop{};
    AwaitSensorNode loop_start_sens{LOOP_START_SID};
    PathToNode path_to_loop_start{LOOP_START_NODE};

    Sequence spd_seq{};
    SetSpeed max_speed1{14};
    Repeat loop_1{&loop_start_sens, 3};
    CalculateSteadySpeed steady_state_speed{};
    Repeat measure_speed{&spd_seq, 1};

    Sequence stop_seq{};
    SetSpeed crawl_speed2{CRAWL_SPEED};
    Repeat loop_5{&loop_start_sens, 1};
    CalculateStop calculate_stop{};
    Repeat measure_stop{&stop_seq, 1};

    Sequence acc_seq{};
    SetSpeed max_speed2{14};
    Repeat loop_3{&loop_start_sens, 1};
    CalculateAccel calculate_accel{};
    Repeat measure_acc{&acc_seq, 1};

    PrintTrainStats print_train_stats{};

    Fallback tree{};
    SetSpeed done_speed{0};
    Invert invert_done_speed{&done_speed};

    CalibrateTrain(uint16_t speed) : max_speed1(speed), max_speed2(speed) {

      // after this we know where we are, and have a
      test_seq.children.push(&localize_tree);

      // ensure we're always pathing in a loop
      test_seq.children.push(&path_to_loop_start);

      // set max speed, loop 2 times, use second for speed
      spd_seq.children.push(&max_speed1);
      spd_seq.children.push(&loop_1);
      spd_seq.children.push(&steady_state_speed);
      test_seq.children.push(&measure_speed);

      // going at max speed
      stop_seq.children.push(&crawl_speed2);
      stop_seq.children.push(&loop_5);
      stop_seq.children.push(&calculate_stop);
      test_seq.children.push(&measure_stop);

      // going at crawl speed
      acc_seq.children.push(&max_speed2);
      acc_seq.children.push(&loop_3);
      acc_seq.children.push(&calculate_accel);
      test_seq.children.push(&measure_acc);

      // done, zero speed & print stats
      test_seq.children.push(&done_speed);
      test_seq.children.push(&print_train_stats);

      // try to do the test sequence
      // if fail, set speed to 0
      tree.children.push(&test_seq);
      tree.children.push(&invert_done_speed);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
  };

  struct CalibrateTrainAllSpeeds : public LeafNode {
    Sequence tree{};
    CalibrateTrain speed_1{1};
    Repeat do_speed_1{&speed_1, 1};
    CalibrateTrain speed_2{2};
    Repeat do_speed_2{&speed_2, 1};
    CalibrateTrain speed_3{3};
    Repeat do_speed_3{&speed_3, 1};
    CalibrateTrain speed_4{4};
    Repeat do_speed_4{&speed_4, 1};
    CalibrateTrain speed_5{5};
    Repeat do_speed_5{&speed_5, 1};
    CalibrateTrain speed_6{6};
    Repeat do_speed_6{&speed_6, 1};
    CalibrateTrain speed_7{7};
    Repeat do_speed_7{&speed_7, 1};
    CalibrateTrain speed_8{8};
    Repeat do_speed_8{&speed_8, 1};
    CalibrateTrain speed_9{9};
    Repeat do_speed_9{&speed_9, 1};
    CalibrateTrain speed_10{10};
    Repeat do_speed_10{&speed_10, 1};
    CalibrateTrain speed_11{11};
    Repeat do_speed_11{&speed_11, 1};
    CalibrateTrain speed_12{12};
    Repeat do_speed_12{&speed_12, 1};
    CalibrateTrain speed_13{13};
    Repeat do_speed_13{&speed_13, 1};
    CalibrateTrain speed_14{14};
    Repeat do_speed_14{&speed_14, 1};

    CalibrateTrainAllSpeeds() {
      tree.children.push(&do_speed_14);
      tree.children.push(&do_speed_13);
      tree.children.push(&do_speed_12);
      tree.children.push(&do_speed_11);
      tree.children.push(&do_speed_10);
      tree.children.push(&do_speed_9);
      tree.children.push(&do_speed_8);
      tree.children.push(&do_speed_7);
      tree.children.push(&do_speed_6);
      tree.children.push(&do_speed_5);
      // tree.children.push(&do_speed_4);
      // tree.children.push(&do_speed_3);
      // tree.children.push(&do_speed_2);
      // tree.children.push(&do_speed_1);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
  };

  struct NavigateTree : public TreeNode {
    Sequence seq{};

    LocalizerTree localizer_tree{};
    UpdateModel update_model{};
    PathToNode path_to_goal{};
    Repeat path_to_goal_once{&path_to_goal, 1};
    LocalizerNode path_localizer{};
    PathLookaheadNode path_lookahead{};
    SetTargetSpeed max_speed{7};
    StopAtDonePath stop_at_done{};

    NavigateTree() {
      seq.children.push(&localizer_tree);
      seq.children.push(&path_to_goal_once);
      seq.children.push(&max_speed);
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
            bb.loco         = bb.state.get_loco(bb.loco_id);

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

void train_tree_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);

  CalibrateTrainAllSpeeds tree{};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .pathfinder{TRACK},
  };
  run_tree(tree, bb);
  std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
}

void nav_tree_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);

  NavigateTree tree{};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .pathfinder{TRACK},
  };
  run_tree(tree, bb);
  std::ignore = send<TC::Ack>(tcs_tid, TC::TreeExit{});
}
