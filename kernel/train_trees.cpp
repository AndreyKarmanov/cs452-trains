#include "train_trees.h"
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
  constexpr auto TRACK              = 'b';
  constexpr auto LOOP_START_NODE    = "C10";
  constexpr int LOOP_START_SID      = sid('C', 10);
  constexpr int LOOP_START_NODE_IDX = LOOP_START_SID - 1;
  constexpr int E3_SID              = sid('E', 3);
  constexpr int E6_SID              = sid('E', 6);
  constexpr size_t CRAWL_SPEED      = 4;

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

  struct DebugPrintDists : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks, spd, tticks");
      auto ttl_ticks = 0;
      for (auto &dist : bb.dists) {
        Debug_Puts(bb.txs_tid, dist.from_sid, ", ",
                   (char)('A' + dist.sensor_data.bank), dist.sensor_data.number,
                   ", ", dist.dx_um / 1000, ", ", dist.d_ticks, ", ",
                   dist.dx_um / dist.d_ticks, ", ", ttl_ticks);
        ttl_ticks += dist.d_ticks;
      }
      return NodeResult::Success;
    }
  };

  struct PrintTrainStats : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      // print all the acceleration values and top speed values for the train
      Debug_Puts(bb.txs_tid, "Train ", bb.loco_id, " Stats: ");
      Debug_Puts(bb.txs_tid, "Speed,Top,Accel,Decel,Stop Dist");
      for (int i = 0; i < 15; ++i) {
        StaticString<128> stats_str{};
        AppendPadded(stats_str, i, 3);
        stats_str.append("   , ");
        AppendPadded(stats_str, bb.loco->v_max_umpt[i], 3);
        stats_str.append(" , ");
        AppendPadded(stats_str, bb.loco->a_nmpt2[i], 5);
        stats_str.append(" , ");
        AppendPadded(stats_str, bb.loco->d_nmpt2[i], 5);
        stats_str.append(" , ", bb.loco->stop_dist_um[i] / 1000);
        Debug_Puts(bb.txs_tid, stats_str);
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

      Debug_Puts(bb.txs_tid, "TOP SPEED: ", bb.target_speed);
      Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
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
          // Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
          //            (*it).dx_um / 1000, ", ", (*it).d_ticks);
        }
      }

      auto estimated_speed                 = ttl_dist_um / ttl_ticks;
      bb.loco->v_max_umpt[bb.target_speed] = estimated_speed;

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

      Debug_Puts(bb.txs_tid, "ACCEL: ", bb.target_speed);
      Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
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

          Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
                     (*it).dx_um / 1000, ", ", (*it).d_ticks);
        }
      }

      auto vc = bb.loco->v_max_umpt[CRAWL_SPEED];
      auto vf = bb.loco->v_max_umpt[bb.target_speed];

      auto accel = (((vf * vf + vc * vc) - 2 * vf * vc) * 1000) /
                   (2 * (vf * ttl_ticks - ttl_dist_um));

      bb.loco->a_nmpt2[bb.target_speed] = accel;

      Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
                 " Total dist: ", ttl_dist_um / 1000,
                 "mm, total ticks: ", ttl_ticks,
                 " acceleration: ", bb.loco->a_nmpt2[bb.target_speed],
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

      Debug_Puts(bb.txs_tid, "STOP: ", bb.target_speed);
      Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
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

          Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
                     (*it).dx_um / 1000, ", ", (*it).d_ticks);
        }
      }

      auto vc = bb.loco->v_max_umpt[CRAWL_SPEED];
      auto vf = bb.loco->v_max_umpt[bb.target_speed];

      auto decel = ((vf * vf - 2 * vf * vc + vc * vc) * 1000) /
                   (2 * (ttl_dist_um - vc * ttl_ticks));

      auto stop_dist                         = ttl_dist_um - vc * ttl_ticks;
      bb.loco->d_nmpt2[bb.target_speed]      = decel;
      bb.loco->stop_dist_um[bb.target_speed] = stop_dist;

      Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
                 " Total dist: ", ttl_dist_um / 1000,
                 "mm, total ticks: ", ttl_ticks, " deceleration: ", decel,
                 "nm/tick^2 stop dist: ", stop_dist / 1000,
                 "mm sensors: ", measurements_used);

      return NodeResult::Success;
    }
  };

  struct SetSpeed : public LeafNode {
    uint16_t req_speed{0};
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
    bool reached_speed = false;
    bool sent_cmd      = false;
    SetTargetSpeed() {}
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
    bool backward{false};
    bool sent_cmd{false};
    SetDirectionNode(bool backward) : backward(backward) {}
    NodeResult tick(Blackboard &bb) override {

      if (auto data = std::get_if<DirectionCmd>(&bb.new_event);
          sent_cmd && data && data->backward == backward) {
        return NodeResult::Success;
      } else if (sent_cmd) {
        return NodeResult::Running;
      }

      auto resp =
          send<TC::Ack>(bb.tcs_tid, TC::Cmd::Direction{.id       = bb.loco_id,
                                                       .backward = backward});
      sent_cmd = true;
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

  uint64_t isqrt(uint64_t n) {
    if (n == 0) {
      return 0;
    }
    uint64_t x = n;
    uint64_t y = (x + 1) / 2;
    while (y < x) {
      x = y;
      y = (x + n / x) / 2;
    }
    return x;
  }

  int predict_ticks(int dist_um, int v_i_nm, const TrainState &loco) {
    int v_m_nm      = loco.v_max_umpt[loco.req_speed] * 1000;
    int64_t dist_nm = static_cast<int64_t>(dist_um) * 1000;

    if (v_m_nm >= v_i_nm) {
      int a = loco.a_nmpt2[loco.req_speed];
      if (a <= 0) {
        return v_i_nm > 0 ? static_cast<int>(dist_nm / v_i_nm) : 0;
      }
      int t_ramp        = (v_m_nm - v_i_nm) / a;
      int64_t d_ramp_nm = (static_cast<int64_t>(a) * t_ramp * t_ramp) / 2 +
                          static_cast<int64_t>(v_i_nm) * t_ramp;
      if (dist_nm <= d_ramp_nm) {
        // solve (a/2) t^2 + v_i t - dist = 0
        int64_t disc = static_cast<int64_t>(v_i_nm) * v_i_nm +
                       2 * static_cast<int64_t>(a) * dist_nm;
        return static_cast<int>((isqrt(disc) - v_i_nm) / a);
      }
      int64_t rem_nm   = dist_nm - d_ramp_nm;
      int64_t t_cruise = v_m_nm > 0 ? rem_nm / v_m_nm : 0;
      return static_cast<int>(t_ramp + t_cruise);
    }

    int tmp_um        = v_i_nm / 1000;
    int model_d       = (3000 + 4200 * tmp_um - 3 * tmp_um * tmp_um) / 10000;
    int d             = std::max(model_d, 33);
    int t_ramp        = (v_i_nm - v_m_nm) / d;
    int64_t d_ramp_nm = static_cast<int64_t>(t_ramp) * (v_i_nm + v_m_nm) / 2;
    if (dist_nm <= d_ramp_nm) {
      // solve (d/2) t^2 - v_i t + dist = 0
      int64_t disc = static_cast<int64_t>(v_i_nm) * v_i_nm -
                     2 * static_cast<int64_t>(d) * dist_nm;
      if (disc < 0) {
        disc = 0;
      }
      return static_cast<int>((v_i_nm - isqrt(disc)) / d);
    }
    int64_t rem_nm   = dist_nm - d_ramp_nm;
    int64_t t_cruise = v_m_nm > 0 ? rem_nm / v_m_nm : 0;
    return static_cast<int>(t_ramp + t_cruise);
  }

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
            path_str.append(bb.pathfinder.track[node.node_idx].name,
                            node.type == NODE_BRANCH
                                ? node.should_br_be_curved ? "C " : "S "
                                : " ",
                            node.dx_next, " >");
          }
          Debug_Puts(bb.txs_tid, path_str);
          return NodeResult::Failure;
        }

        auto dx_mm = std::accumulate(
            bb.path.begin(), idx + 1, 0,
            [](int acc, const PathNode &node) { return acc + node.dx_prev; });

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

        // reset distance to next sensor after pop
        bb.dx_um = 0;

        if (bb.path.empty()) {
          bb.pending.active = false;
        } else {
          if (bb.pending.active) {
            int dt    = static_cast<int>(bb.curr_tick) -
                        static_cast<int>(bb.pending.predicted_tick);
            int dx_um = (bb.pending.v_at_prediction_nm / 1000) * dt;
            // Debug_Puts(
            //     bb.txs_tid, "sensor ", (char)('A' + data->bank),
            //     data->number, " reached: ", "time error
            //     (t_actual-t_predicted) = ", dt, " ticks | distance error
            //     (v*dt) = ", dx_um / 1000, " mm (v = ",
            //     bb.pending.v_at_prediction_nm / 1000, " um/tick)");
            bb.pending.active = false;
          }

          auto next_sens = std::ranges::find(
              bb.path, true, [](PathNode &n) { return n.type == NODE_SENSOR; });
          if (next_sens != bb.path.end()) {
            int dist_mm = std::accumulate(
                bb.path.begin(), next_sens + 1, 0,
                [](int acc, const PathNode &n) { return acc + n.dx_prev; });
            bb.pending = {
                .active = true,
                .predicted_tick =
                    bb.curr_tick +
                    static_cast<uint32_t>(predict_ticks(
                        dist_mm * 1000, bb.loco->ve_nm, *bb.loco)),
                .v_at_prediction_nm = bb.loco->ve_nm,
            };
          }
        }

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

      int v_m_nm = bb.loco->v_max_umpt[bb.loco->req_speed] * 1000;
      int v_i_nm = bb.loco->ve_nm;
      int tmp_um = v_i_nm / 1000;
      int d_t    = bb.curr_tick - bb.last_tick;

      auto delta = 0;
      if (v_m_nm >= v_i_nm) {
        int a          = bb.loco->a_nmpt2[bb.loco->req_speed];
        int t_a        = std::min((v_m_nm - v_i_nm) / a, d_t);
        bb.loco->ve_nm = v_i_nm + a * t_a;

        int t_c = d_t - t_a;
        delta   = ((a * t_a * t_a) / 2 + v_m_nm * t_c + v_i_nm * t_a) / 1000;
      } else {
        int model_d    = (3000 + 4200 * tmp_um - 3 * tmp_um * tmp_um) / 10000;
        int d          = std::max(model_d, 33);
        int t_d        = std::min((v_i_nm - v_m_nm) / d, d_t);
        bb.loco->ve_nm = v_i_nm - d * t_d;

        delta = (t_d * (v_i_nm - v_m_nm) / 2 + v_m_nm * d_t) / 1000;
      }

      bb.dx_um += delta;

      Offset_Puts(bb.txs_tid, -2, "Spd: ", bb.loco->ve_nm / 1000, "um/ms d_t ",
                  d_t, " v_i ", v_i_nm / 1000, " v_max ", v_m_nm / 1000,
                  "nm/t^2 dx_mm", bb.dx_um / 1000, "\033[K");
      return NodeResult::Success;
    }
  };

  struct PathLookaheadNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        return NodeResult::Success;
      }

      auto lookahead_um =
          bb.dx_um +
          static_cast<uint32_t>(bb.loco->ve_nm / 1000) * TICKS_PER_S * 3;
      lookahead_um =
          lookahead_um > 1500u * 1000u ? lookahead_um : 1500u * 1000u;

      // calculate distance travelled given current velocity
      // safe estimate is max velocity for speed
      // then, calculate distance based on velocity. suppose distance is 500
      uint32_t total_dist = 0;
      for (auto &node : bb.path) {
        total_dist += node.dx_prev;
        if (total_dist * 1000 > (lookahead_um)) {
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

    SetSpeed stop{0};
    int offset_mm{0};

    StopAtDonePath() = default;
    StopAtDonePath(int offset_mm) : offset_mm(offset_mm) {}

    NodeResult tick(Blackboard &bb) override {

      if (bb.path.empty()) {
        return stop.tick(bb);
      }

      auto remaining_dist_um =
          std::accumulate(bb.path.begin(), bb.path.end(), 0,
                          [](int acc, const PathNode &node) {
                            return acc + node.dx_prev;
                          }) *
              1000 -
          bb.dx_um + offset_mm * 1000;

      auto x               = bb.loco->ve_nm / 1000;
      int stopping_dist_um = 26000 + 582 * x + 3.4 * x * x;
      Offset_Puts(bb.txs_tid, -3, "D: ", remaining_dist_um / 1000,
                  "mm sd: ", stopping_dist_um / 1000, "mm");

      if (remaining_dist_um < stopping_dist_um) {
        stop.tick(bb);
      }
      return NodeResult::Running;
    };
  };

  struct PathToNode : public LeafNode {
    int goal_idx{};
    PathToNode(int goal_idx) : goal_idx(goal_idx) {}

    NodeResult tick(Blackboard &bb) override {
      if (bb.seen_sensors.empty() && bb.path.empty()) {
        bb.error_msg = "Failed to find start";
        return NodeResult::Failure;
      }

      auto start_idx =
          bb.seen_sensors.peek_last().has_value()
              ? bb.seen_sensors.peek_last()->sid - 1 // sid -1 is it's node_idx
              : bb.path.peek_last().value().node_idx;

      auto last_node = bb.path.peek_last();
      if (last_node.has_value() && last_node->node_idx == goal_idx) {
        return NodeResult::Success;
      }

      auto path_opt = bb.pathfinder.shortest_path(start_idx, goal_idx);

      if (!path_opt.has_value()) {
        Debug_Puts(bb.txs_tid, "Can't path ", start_idx, " to ", goal_idx);
        return NodeResult::Failure;
      }
      bb.path = path_opt.value();
      return NodeResult::Success;
    }
  };

  struct LocalizerTree : public LeafNode {
    Sequence tree{};

    Sequence inital{};
    SetSpeed set_speed{CRAWL_SPEED};
    AwaitSensorNode await_sensor{};
    bool initalized{false};

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

    NodeResult tick(Blackboard &bb) override {
      if (!initalized) {
        if (bb.loco->req_speed != CRAWL_SPEED && bb.loco->req_speed != 0) {
          set_speed = SetSpeed{bb.loco->req_speed};
        }
        initalized = true;
      }
      return tree.tick(bb);
    }
  };

  struct CalibrateTrain : public LeafNode {

    uint16_t target_speed{0};
    bool done = false;

    Sequence test_seq{};
    LocalizerTree localize_tree{};

    Sequence setup_loop{};
    AwaitSensorNode loop_start_sens{LOOP_START_SID};
    PathToNode path_to_loop_start{LOOP_START_NODE_IDX};

    Sequence spd_seq{};
    SetTargetSpeed max_speed1{};
    Repeat loop_1{&loop_start_sens, 3};
    CalculateSteadySpeed steady_state_speed{};
    Repeat measure_speed{&spd_seq, 1};

    Sequence stop_seq{};
    SetSpeed crawl_speed2{CRAWL_SPEED};
    Repeat loop_5{&loop_start_sens, 1};
    CalculateStop calculate_stop{};
    Repeat measure_stop{&stop_seq, 1};

    Sequence acc_seq{};
    SetTargetSpeed max_speed2{};
    Repeat loop_3{&loop_start_sens, 1};
    CalculateAccel calculate_accel{};
    Repeat measure_acc{&acc_seq, 1};

    PrintTrainStats print_train_stats{};
    DebugPrintDists debug_print_dists{};

    Fallback tree{};
    SetSpeed done_speed{0};
    Invert invert_done_speed{&done_speed};

    CalibrateTrain(uint16_t speed) : target_speed(speed) {

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
      test_seq.children.push(&debug_print_dists);
      test_seq.children.push(&print_train_stats);

      // try to do the test sequence
      // if fail, set speed to 0
      tree.children.push(&test_seq);
      tree.children.push(&invert_done_speed);
    }

    NodeResult tick(Blackboard &bb) override {
      if (!done && target_speed != 0 && bb.target_speed != target_speed) {
        bb.target_speed = target_speed;
      }
      auto res = tree.tick(bb);
      done     = res == NodeResult::Success || res == NodeResult::Failure;
      return res;
    }
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

    std::array<CalibrateTrain *, 14> speeds{
        &speed_1,  &speed_2,  &speed_3,  &speed_4, &speed_5,
        &speed_6,  &speed_7,  &speed_8,  &speed_9, &speed_10,
        &speed_11, &speed_12, &speed_13, &speed_14};

    int cal_speed = 0;

    CalibrateTrainAllSpeeds(int cal_speed) : cal_speed(cal_speed) {
      // tree.children.push(&do_speed_2);
      // tree.children.push(&do_speed_3);

      // crawl speed first
      tree.children.push(&do_speed_4);

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
    }

    NodeResult tick(Blackboard &bb) override {
      if (cal_speed == 0) {
        return tree.tick(bb);
      } else if (cal_speed > 0 && cal_speed <= 14) {
        return speeds[cal_speed - 1]->tick(bb);
      } else {
        bb.error_msg = "Invalid cal_speed value";
        return NodeResult::Failure;
      }
    }
  };

  struct StopMeasureTree : public LeafNode {
    Sequence tree{};

    LocalizerTree localize_tree{};

    PathToNode path_to_e3{sid('E', 3) - 1};
    // repeat 1x so that it only sets path to e3 once
    Repeat path_to_e3_once{&path_to_e3, 1};
    SetSpeed crawl_to_e3{14};
    AwaitSensorNode arrive_e3{E3_SID};
    // repeat 1x so that it only waits for sensor once
    Repeat arrive_e3_once{&arrive_e3, 1};
    Sequence pre_loop{};

    Sequence warmup{};
    SetSpeed set_speed{0};
    AwaitSensorNode warmup_sens{E3_SID};
    Repeat warmup_laps{&warmup, 1};

    Sequence approach{};
    PathToNode path_to_D4{sid('D', 4) - 1};

    Sequence stop_on_sens{};
    AwaitSensorNode stop_sens{E6_SID};
    Repeat stop_sens_once{&stop_sens, 1};
    SetSpeed stop_speed{0};

    bool initalized{false};

    StopMeasureTree(uint16_t target_speed) : set_speed{target_speed} {
      pre_loop.children.push(&path_to_e3_once);
      pre_loop.children.push(&crawl_to_e3);
      pre_loop.children.push(&arrive_e3_once);

      warmup.children.push(&set_speed);
      warmup.children.push(&warmup_sens);

      approach.children.push(&path_to_D4);

      stop_on_sens.children.push(&stop_sens_once);
      stop_on_sens.children.push(&stop_speed);

      tree.children.push(&localize_tree);
      tree.children.push(&pre_loop);
      tree.children.push(&warmup_laps);
      tree.children.push(&approach);
      tree.children.push(&stop_on_sens);
    }

    NodeResult tick(Blackboard &bb) override { return tree.tick(bb); }
  };

  struct NavigateTree : public TreeNode {
    Sequence seq{};

    LocalizerTree localizer_tree{};
    UpdateModel update_model{};
    PathToNode path_to_goal{sid('D', 4) - 1};
    Repeat path_to_goal_once{&path_to_goal, 1};
    LocalizerNode path_localizer{};
    PathLookaheadNode path_lookahead{};
    SetSpeed max_speed{7};
    StopAtDonePath stop_at_done{};
    DebugPrintDists debug_print_dists{};
    PrintTrainStats print_train_stats{};

    NavigateTree(int goal_idx, uint16_t speed, int offset_mm)
        : path_to_goal{goal_idx}, max_speed{speed}, stop_at_done{offset_mm} {
      seq.children.push(&localizer_tree);
      seq.children.push(&path_to_goal_once);
      seq.children.push(&max_speed);
      seq.children.push(&stop_at_done);
      seq.children.push(&debug_print_dists);
    }

    NodeResult tick(Blackboard &bb) override {
      if (!bb.error_msg.empty()) {
        return NodeResult::Failure;
      }
      return seq.tick(bb);
    }
  };

  struct ReverseTree : public TreeNode {

    bool initalized{false};
    bool at_speed{false};
    SetSpeed stop_speed{0};
    WaitNode wait_to_stop{7 * TICKS_PER_S};
    SetDirectionNode dir{false};
    SetSpeed set_speed{0};
    Sequence going_seq{};

    ReverseTree() {
      going_seq.children.push(&stop_speed);
      going_seq.children.push(&wait_to_stop);
      going_seq.children.push(&dir);
      going_seq.children.push(&set_speed);
    }

    NodeResult tick(Blackboard &bb) override {
      if (!initalized) {
        dir        = SetDirectionNode{!bb.loco->backward};
        at_speed   = bb.loco->req_speed > 0;
        set_speed  = SetSpeed{bb.loco->req_speed};
        initalized = true;
      }
      if (!at_speed) {
        return dir.tick(bb);
      }
      return going_seq.tick(bb);
    }
  };

} // namespace

using Tree = std::variant<CalibrateTrainAllSpeeds, PrintTrainStats,
                          StopMeasureTree, NavigateTree, ReverseTree>;

void run_tree() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);

  Tree tree = PrintTrainStats{};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .pathfinder{TRACK},
  };

  TrainState tmp{};
  bb.loco = &tmp;
  while (true) {
    auto next_msg =
        send<TC::Tree::Msg>(bb.tcs_tid, TC::Tree::Ready{.train = *bb.loco});
    if (!next_msg.has_value()) {
      break;
    }

    auto msg_result = std::visit(
        [&](auto &&event) {
          using Event = std::decay_t<decltype(event)>;
          if constexpr (std::is_same_v<Event, TC::Tree::Init>) {
            bb.state   = event.state;
            bb.loco_id = event.loco_id;
            bb.loco    = bb.state.get_loco(bb.loco_id);

            switch (event.tree_type) {
            case TC::Tree::Type::CALIBRATE:
              tree.emplace<CalibrateTrainAllSpeeds>(event.value1);
              break;
            case TC::Tree::Type::PRINT_TRAIN_STATS:
              tree.emplace<PrintTrainStats>();
              break;
            case TC::Tree::Type::STOP_MEASURE:
              tree.emplace<StopMeasureTree>(event.value1);
              break;
            case TC::Tree::Type::REVERSE:
              tree.emplace<ReverseTree>();
              break;
            case TC::Tree::Type::NAVIGATE: {
              tree.emplace<NavigateTree>(event.value1, event.value2,
                                         event.value3);
              break;
            }
            default: {
              bb.error_msg = "Unknown tree type";
              return false;
            }
            }
            return true;
          } else if constexpr (std::is_same_v<Event, TC::Tree::Update>) {
            bb.state.update_from_mrk(event.mrk, event.time);
            bb.new_event = event.mrk;
            bb.last_tick = bb.curr_tick;
            bb.curr_tick = event.time;
          } else {
            bb.error_msg = "Unknown event type";
            return false;
          }
          return true;
        },
        next_msg.value());
    if (!msg_result) {
      Debug_Puts(bb.txs_tid, "Error: ", bb.error_msg);
      break;
    }

    auto result = std::visit([&](auto &t) { return t.tick(bb); }, tree);
    if (result == NodeResult::Failure) {
      Debug_Puts(bb.txs_tid, "Error: ", bb.error_msg);
      break;
    } else if (result == NodeResult::Success) {
      break;
    }
  }
  std::ignore = send<TC::Ack>(tcs_tid, TC::Tree::Exit{});
}