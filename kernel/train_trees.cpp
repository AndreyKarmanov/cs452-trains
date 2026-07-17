#include "train_trees.h"
#include "behaviour_tree.h"
#include "io_helpers.h"
#include "message.h"
#include "mrk.h"
#include "overloaded.h"
#include "pathfind.h"
#include "rng.h"
#include "time.h"
#include "track_data.h"
#include "train_control.h"
#include <algorithm>
#include <cstdint>
#include <iterator>
#include <numeric>
#include <optional>
#include <ranges>

namespace {
  auto sid = [](char b, int n) -> uint16_t { return (b - 'A') * 16 + n; };
  constexpr auto TRACK_LAYOUT       = Track::Layout::A;
  constexpr auto LOOP_START_NODE    = "C12";
  constexpr int LOOP_START_SID      = sid('C', 12);
  constexpr int LOOP_START_NODE_IDX = LOOP_START_SID - 1;
  constexpr int E3_SID              = sid('E', 3);
  constexpr int E6_SID              = sid('E', 6);
  constexpr size_t CRAWL_SPEED      = 4;

  void print_dists(int txs_tid,
                   const Buffer<Blackboard::DistLog, TRACK_MAX> &dists) {
    Debug_Puts(txs_tid, "from, to, dist (mm), ticks, spd, tticks");
    auto ticks = 0;
    for (auto &dist : dists) {
      Debug_Puts(txs_tid, dist.from_sid, ", ",
                 (char)('A' + dist.sensor_data.bank), dist.sensor_data.number,
                 ", ", dist.dx_um / 1000, ", ", dist.d_ticks, ", ",
                 dist.dx_um / dist.d_ticks, ", ", ticks);
      ticks += dist.d_ticks;
    }
  }

  struct DebugPrintPath : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      StaticString<128> path_str{};
      path_str.append("Path: ", bb.path.size(), " ");
      for (const auto &node : bb.path) {
        path_str.append(bb.track[node.node_idx].name,
                        node.type == NODE_BRANCH
                            ? node.should_br_be_curved ? "C " : "S "
                            : " ",
                        node.dx_next, " >");
      }
      Debug_Puts(bb.txs_tid, path_str);
      return NodeResult::Success;
    }
  };

  struct DebugPrintDists : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      print_dists(bb.txs_tid, bb.dists);
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

  struct PrintLastLoopDists : public LeafNode {
    uint16_t loops_to_use{1};
    uint16_t loop_start_sid{LOOP_START_SID};

    PrintLastLoopDists(uint16_t n = 1, uint16_t loop_start_sid = LOOP_START_SID)
        : loops_to_use(n), loop_start_sid(loop_start_sid) {}

    NodeResult tick(Blackboard &bb) override {
      bool found_start = false;
      int sensors      = 0;

      uint32_t dist_um = 0;
      uint32_t ticks   = 0;

      Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
      for (auto it = bb.dists.end() - 1;
           it != bb.dists.begin() && loops_to_use > 0; it--) {
        if ((*it).sensor_data.sensor_id == loop_start_sid) {
          if (found_start == false) {
            found_start = true;
          } else {
            loops_to_use--;
          }
        }
        if (found_start && loops_to_use > 0) {
          dist_um += (*it).dx_um;
          ticks   += (*it).d_ticks;
          sensors++;
          Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
                     (*it).dx_um / 1000, ", ", (*it).d_ticks);
        }
      }

      Debug_Puts(bb.txs_tid, "dist(mm), ticks, speed(um/t), sensors\n\r",
                 dist_um / 1000, ticks, dist_um / ticks, sensors);
      return NodeResult::Success;
    }
  };

  // struct CalculateSteadySpeed : public LeafNode {
  //   uint16_t loop_start_sid{LOOP_START_SID};

  //   CalculateSteadySpeed(uint16_t loop_start_sid = LOOP_START_SID)
  //       : loop_start_sid(loop_start_sid) {}

  //   NodeResult tick(Blackboard &bb) override {
  //     bool found_start      = false;
  //     int loops_to_use      = 2;
  //     int measurements_used = 0;

  //     uint32_t ttl_dist_um = 0;
  //     uint32_t ttl_ticks   = 0;

  //     Debug_Puts(bb.txs_tid, "TOP SPEED: ", bb.target_speed);
  //     Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
  //     for (auto it = bb.dists.end() - 1;
  //          it != bb.dists.begin() && loops_to_use > 0; it--) {
  //       if ((*it).sensor_data.sensor_id == loop_start_sid) {
  //         if (found_start == false) {
  //           found_start = true;
  //         } else {
  //           loops_to_use--;
  //         }
  //       }
  //       if (found_start && loops_to_use > 0) {
  //         ttl_dist_um += (*it).dx_um;
  //         ttl_ticks   += (*it).d_ticks;
  //         measurements_used++;
  //         Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
  //                    (*it).dx_um / 1000, ", ", (*it).d_ticks);
  //       }
  //     }

  //     auto estimated_speed                 = ttl_dist_um / ttl_ticks;
  //     bb.loco->v_max_umpt[bb.target_speed] = estimated_speed;

  //     Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
  //                " Total dist: ", ttl_dist_um / 1000,
  //                "mm, total ticks: ", ttl_ticks, " speed: ", estimated_speed,
  //                "um/tick Sensors used: ", measurements_used, "");
  //     return NodeResult::Success;
  //   }
  // };

  // struct CalculateAccel : public LeafNode {
  //   NodeResult tick(Blackboard &bb) override {
  //     bool found_start      = false;
  //     int loops_to_use      = 1;
  //     int measurements_used = 0;

  //     uint32_t ttl_dist_um = 0;
  //     uint32_t ttl_ticks   = 0;

  //     Debug_Puts(bb.txs_tid, "ACCEL: ", bb.target_speed);
  //     Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
  //     for (auto it = bb.dists.end() - 1;
  //          it != bb.dists.begin() && loops_to_use > 0; it--) {
  //       if ((*it).sensor_data.sensor_id == LOOP_START_SID) {
  //         if (found_start == false) {
  //           found_start = true;
  //         } else {
  //           loops_to_use--;
  //         }
  //       }
  //       if (found_start && loops_to_use > 0) {
  //         ttl_dist_um += (*it).dx_um;
  //         ttl_ticks   += (*it).d_ticks;
  //         measurements_used++;

  //         Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
  //                    (*it).dx_um / 1000, ", ", (*it).d_ticks);
  //       }
  //     }

  //     auto vc = bb.loco->v_max_umpt[CRAWL_SPEED];
  //     auto vf = bb.loco->v_max_umpt[bb.target_speed];

  //     auto accel = (((vf * vf + vc * vc) - 2 * vf * vc) * 1000) /
  //                  (2 * (vf * ttl_ticks - ttl_dist_um));

  //     bb.loco->a_nmpt2[bb.target_speed] = accel;

  //     Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
  //                " Total dist: ", ttl_dist_um / 1000,
  //                "mm, total ticks: ", ttl_ticks,
  //                " acceleration: ", bb.loco->a_nmpt2[bb.target_speed],
  //                "nm/tick^2 sensors: ", measurements_used);

  //     return NodeResult::Success;
  //   }
  // };

  // struct CalculateStop : public LeafNode {
  //   NodeResult tick(Blackboard &bb) override {
  //     bool found_start      = false;
  //     int loops_to_use      = 1;
  //     int measurements_used = 0;

  //     uint32_t ttl_dist_um = 0;
  //     uint32_t ttl_ticks   = 0;

  //     Debug_Puts(bb.txs_tid, "STOP: ", bb.target_speed);
  //     Debug_Puts(bb.txs_tid, "from, to, dist (mm), ticks");
  //     for (auto it = bb.dists.end() - 1;
  //          it != bb.dists.begin() && loops_to_use > 0; it--) {
  //       if ((*it).sensor_data.sensor_id == LOOP_START_SID) {
  //         if (found_start == false) {
  //           found_start = true;
  //         } else {
  //           loops_to_use--;
  //         }
  //       }
  //       if (found_start && loops_to_use > 0) {
  //         ttl_dist_um += (*it).dx_um;
  //         ttl_ticks   += (*it).d_ticks;
  //         measurements_used++;

  //         Debug_Puts(bb.txs_tid, (*it).from_sid, ", ", (*it).to_sid, ", ",
  //                    (*it).dx_um / 1000, ", ", (*it).d_ticks);
  //       }
  //     }

  //     auto vc = bb.loco->v_max_umpt[CRAWL_SPEED];
  //     auto vf = bb.loco->v_max_umpt[bb.target_speed];

  //     auto decel = ((vf * vf - 2 * vf * vc + vc * vc) * 1000) /
  //                  (2 * (ttl_dist_um - vc * ttl_ticks));

  //     auto stop_dist                         = ttl_dist_um - vc * ttl_ticks;
  //     bb.loco->d_nmpt2[bb.target_speed]      = decel;
  //     bb.loco->stop_dist_um[bb.target_speed] = stop_dist;

  //     Debug_Puts(bb.txs_tid, "Speed ", bb.target_speed,
  //                " Total dist: ", ttl_dist_um / 1000,
  //                "mm, total ticks: ", ttl_ticks, " deceleration: ", decel,
  //                "nm/tick^2 stop dist: ", stop_dist / 1000,
  //                "mm sensors: ", measurements_used);

  //     return NodeResult::Success;
  //   }
  // };

  struct SetSpeed : public LeafNode {
    uint16_t req_speed{0};
    bool reached_speed = false;
    bool sent_cmd      = false;
    SetSpeed(uint16_t speed) : req_speed(speed) {}
    NodeResult tick(Blackboard &bb) override {
      if (reached_speed || (sent_cmd && bb.loco->req_speed == req_speed)) {
        reached_speed = true;
        return NodeResult::Success;
      }

      if (!sent_cmd) {
        auto resp = send<TC::Ack>(
            bb.tcs_tid, TC::Cmd::Speed{.id = bb.loco_id, .value = req_speed});
        if (!resp.has_value()) {
          bb.error_msg = "Failed to set speed";
          return NodeResult::Failure;
        }
        sent_cmd = true;
      }

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

  struct UpdateModel : public LeafNode {
    uint64_t last_tick{0};
    uint64_t last_print{0};
    uint16_t decel_from_speed{0};

    NodeResult tick(Blackboard &bb) override {
      if (last_tick == 0) {
        last_tick = bb.curr_tick;
        return NodeResult::Success;
      }

      uint64_t v_m_nm = bb.loco->v_max_umpt[bb.loco->req_speed] * 1000 *
                        (TICK_TIME_US / 1'000);
      uint64_t v_i_nm = bb.loco->ve_nm;
      uint64_t d_t    = bb.curr_tick - last_tick;
      last_tick       = bb.curr_tick;

      uint64_t delta = 0;
      if (v_m_nm >= v_i_nm) {
        // accelerating (or cruising): constant a until v_max, then cruise
        uint64_t a =
            bb.loco->a_nmpt2[bb.loco->req_speed] * (TICK_TIME_US / 1'000);
        uint64_t t_a   = std::min((v_m_nm - v_i_nm) / a, d_t);
        bb.loco->ve_nm = v_i_nm + a * t_a;

        uint64_t t_c = d_t - t_a;
        delta = ((a * t_a * t_a) / 2 + v_m_nm * t_c + v_i_nm * t_a) / 1000;

        // track which speed level our current velocity corresponds to,
        // so a later slow-down uses the right decel constant
        decel_from_speed = bb.loco->req_speed;
      } else {
        uint64_t d =
            bb.loco->d_nmpt2[decel_from_speed] * (TICK_TIME_US / 1'000);
        uint64_t t_d   = std::min((v_i_nm - v_m_nm) / d, d_t);
        bb.loco->ve_nm = v_i_nm - d * t_d;

        uint64_t t_c = d_t - t_d;
        delta = (v_i_nm * t_d - (d * t_d * t_d) / 2 + v_m_nm * t_c) / 1000;
      }

      bb.dx_um += delta;

#if !defined(DATA_COLLECTION) || !DATA_COLLECTION
      if (bb.curr_tick - last_print > 100) {
        last_print = bb.curr_tick;
        Offset_Puts(bb.txs_tid, -2, "Spd: ", bb.loco->ve_nm / 1000,
                    "um/ms d_t ", d_t, " v_i ", v_i_nm / 1000, " v_max ",
                    v_m_nm / 1000, "nm/t^2 dx_mm", bb.dx_um / 1000, "\033[K");
      }
#endif

      return NodeResult::Success;
    }
  };

  struct AttributeSensorNode : public LeafNode {
    static constexpr int PCT_TOLERANCE = 50;

    void push_to_seen_sensors(Blackboard &bb, SensorData *data) {
      if (bb.seen_sensors.size() == bb.seen_sensors.capacity()) {
        bb.seen_sensors.pop();
      }

      bb.seen_sensors.push({
          .sid  = data->sensor_id,
          .tick = bb.curr_tick,
      });
    }

    NodeResult tick(Blackboard &bb) override {
      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1) {

        // not registered? only one train
        if (bb.loco->inital_node_idx == -1) {
          push_to_seen_sensors(bb, data);
          return NodeResult::Success;
        }

        // if it's our first sensor, wait for the given inital sensor
        if (bb.seen_sensors.empty()) {
          if (bb.loco->inital_node_idx + 1 != data->sensor_id) {
            Debug_Puts(bb.txs_tid, "Ignored Inital: ", data->sensor_id, " ",
                       (char)('A' + data->bank), data->number, " expected ",
                       bb.loco->inital_node_idx + 1);

            return NodeResult::Running;
          }
          Debug_Puts(bb.txs_tid, "First sensor: ", data->sensor_id, " ",
                     (char)('A' + data->bank), data->number);
          push_to_seen_sensors(bb, data);
          return NodeResult::Success;
        }

        // otherwise, check how far we are from the sensor
        // we always use shortest path for travel, so can safely use this dist.
        auto path = bb.track.find_path(bb.seen_sensors.peek_last()->sid - 1,
                                       data->sensor_id - 1);

        // if there's no path, or 20% off our estimate, we ignore
        if (!path.has_value()) {
          Debug_Puts(bb.txs_tid, "Ignored sensor (no path): ", data->sensor_id,
                     " ", (char)('A' + data->bank), data->number);
          return NodeResult::Running;
        }

        auto sens_dist_um = path->dist_mm * 1000;
        if (sens_dist_um > (bb.dx_um * (100 + PCT_TOLERANCE)) / 100 ||
            sens_dist_um < (bb.dx_um * (100 - PCT_TOLERANCE)) / 100) {
          Debug_Puts(bb.txs_tid, "Ignored sensor (out of range): ",
                     (char)('A' + data->bank), data->number, " pos ",
                     bb.dx_um * 100 / sens_dist_um, "% ",
                     (bb.dx_um - sens_dist_um) / 1000, " mm");
          return NodeResult::Running;
        }
        Debug_Puts(bb.txs_tid, "Attributed: ", data->sensor_id, " ",
                   (char)('A' + data->bank), data->number, " pos ",
                   bb.dx_um * 100 / sens_dist_um, "% ",
                   (bb.dx_um - sens_dist_um) / 1000, " mm");
        push_to_seen_sensors(bb, data);
      }
      return NodeResult::Success;
    }
  };

  struct LocalizerNode : public LeafNode {

    DebugPrintPath print_path{};
    NodeResult tick(Blackboard &bb) override {
      if (auto data = std::get_if<SensorData>(&bb.new_event);
          data && data->new_state == 1) {

        if (bb.path.empty()) {
          bb.dx_um = 0;
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
          print_path.tick(bb);
          bb.error_msg = "Sensor not in path";
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
        bb.dx_um = 0;
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
          bb.dx_um +
          static_cast<uint32_t>(bb.loco->ve_nm / 1000) * TICKS_PER_S * 3;
      lookahead_um =
          lookahead_um > 1500u * 1000u ? lookahead_um : 1500u * 1000u;

      auto stopping_dist = bb.loco->stop_dist_um[bb.loco->req_speed];

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
              bb.error_msg = "Switch cmd failed";
              return NodeResult::Failure;
            }
          } else if (!node.should_br_be_curved &&
                     !bb.state.is_switch_straight(node.num)) {
            // Debug_Puts(bb.txs_tid, "Setting switch ", node.num, " to
            // straight");
            auto res =
                send<TC::Ack>(bb.tcs_tid, TC::Cmd::Switch(node.num, true));
            if (!res.has_value()) {
              bb.error_msg = "Switch cmd failed";
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
    bool stopping = false;

    StopAtDonePath() = default;
    StopAtDonePath(int offset_mm) : offset_mm(offset_mm) {}

    NodeResult tick(Blackboard &bb) override {

      // stopping logic:
      // stopping: wait for ve = 0
      // not stopping: don't do anything?
      if (stopping) {
        auto res = stop.tick(bb);
        if (res == NodeResult::Success) {
          if (bb.loco->ve_nm / 1000 > 0) {
            return NodeResult::Running;
          }
          return NodeResult::Success;
        }
        return res;
      }

      if (bb.path.empty()) {
        stopping = true;
        return stop.tick(bb);
      }

      // todo: account for going to a reversed destination (invert offset)
      // todo: account for going in reverse (add offset?)
      auto remaining_mm =
          std::ranges::fold_left(bb.path, 0, [](int acc, const PathNode &node) {
            return acc + node.dx_prev;
          });

      auto remaining_um = remaining_mm * 1000 - bb.dx_um + offset_mm * 1000;

      auto x               = bb.loco->ve_nm / 1000;
      int stopping_dist_um = 26000 + 582 * x + 3.4 * x * x;

      // stopping distance is a linear interpolation between our measured
      // stopping distances

      // auto u_v_m   = bb.loco->v_max_umpt[bb.loco->req_speed];
      // auto u_sd_um = bb.loco->stop_dist_um[bb.loco->req_speed];
      // auto stopping_dist_um = (u_sd_um * bb.loco->ve_nm) / (u_v_m * 1000);
      // auto stopping_dist_um = bb.loco->stop_dist_um[bb.loco->req_speed];

      Offset_Puts(bb.txs_tid, -3, "D: ", remaining_um / 1000,
                  "mm sd: ", stopping_dist_um / 1000, "mm");

      if (remaining_um < stopping_dist_um) {
        stopping = true;
        return stop.tick(bb);
      }

      return NodeResult::Running;
    };
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
        auto res = dir.tick(bb);
        if (res == NodeResult::Success) {
          bb.reversed_since_last_sensor = true;
        }
        return res;
      }
      auto res = going_seq.tick(bb);
      if (res == NodeResult::Success) {
        bb.reversed_since_last_sensor = true;
      }
      return res;
    }
  };

  struct PathToNode : public LeafNode {
    int goal_idx{};

    std::optional<ReverseTree> rev_tree{std::in_place};

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

      auto startr_idx = bb.track.node_idx(bb.track[start_idx].reverse);
      auto goalr_idx  = bb.track.node_idx(bb.track[goal_idx].reverse);

      if (auto last_node = bb.path.peek_last();
          last_node.has_value() && (last_node->node_idx == goal_idx ||
                                    last_node->node_idx == goalr_idx)) {
        return NodeResult::Success;
      }

      bool should_reverse = false;

      // start to ooal
      auto path_opt = bb.track.find_path(start_idx, goal_idx);

      // start to reverse ooal
      if (!path_opt.has_value()) {
        path_opt = bb.track.find_path(start_idx, goalr_idx);
      }

      // reverse start to ooal
      if (!path_opt.has_value()) {
        path_opt       = bb.track.find_path(startr_idx, goal_idx);
        should_reverse = true;
      }

      // reverse start to reverse ooal
      if (!path_opt.has_value()) {
        path_opt       = bb.track.find_path(startr_idx, goalr_idx);
        should_reverse = true;
      }

      if (!path_opt.has_value()) {
        bb.error_msg = "No path to goal";
        return NodeResult::Failure;
      }

      if (should_reverse) {
        auto res = rev_tree->tick(bb);
        if (res == NodeResult::Success) {
          rev_tree.emplace();
        } else {
          return res;
        }
      }

      bb.path                  = bb.path + path_opt.value();
      bb.loco->target_node_idx = goal_idx;
      return NodeResult::Success;
    }
  };

  struct LocalizerTree : public LeafNode {
    UpdateModel model{};
    SetSpeed set_speed{CRAWL_SPEED};
    AttributeSensorNode attribute_sensor{};
    LocalizerNode localize{};
    PathLookaheadNode lookahead{};

    Sequence loop{
        &model, &set_speed, &attribute_sensor, &localize, &lookahead,
    };

    NodeResult tick(Blackboard &bb) override {

      // if we're already moving, keep the same speed
      // otherwise sets to crawl speed to start localizing
      if (bb.seen_sensors.empty() && bb.loco->req_speed != 0 &&
          set_speed.req_speed != bb.loco->req_speed) {
        set_speed = SetSpeed{bb.loco->req_speed};
      }

      auto res = loop.tick(bb);
      if (res == NodeResult::Failure) {
        return NodeResult::Failure;
      } else if (bb.seen_sensors.empty()) {
        return NodeResult::Running;
      }
      return res;
    }
  };

  struct CalibrateTree : public LeafNode {

    constexpr static auto DEFAULT_SPEED    = 10;
    constexpr static auto SLOW_SPEED       = 4;
    constexpr static auto DEFAULT_LOOP_SID = sid('B', 3);

    constexpr static auto TOP_SPEED_LOOPS = 2;
    constexpr static auto ACCEL_LOOPS     = 1;
    constexpr static auto DECEL_LOOPS     = 1;
    constexpr static auto TOTAL_LOOPS =
        TOP_SPEED_LOOPS + ACCEL_LOOPS + DECEL_LOOPS;

    uint16_t cal_speed{DEFAULT_SPEED};
    uint16_t loop_start_sid{DEFAULT_LOOP_SID};

    LocalizerTree localize{};
    SetSpeed test_speed{cal_speed};
    SetSpeed test_speed_2{cal_speed};
    SetSpeed crawl_speed{SLOW_SPEED};
    SetSpeed done_speed{0};

    PathToNode path_in_loop{loop_start_sid - 1};

    AwaitSensorNode await_loop_sid{loop_start_sid};
    Repeat loop_speed{&await_loop_sid, TOP_SPEED_LOOPS + 1};
    Repeat loop_decel{&await_loop_sid, DECEL_LOOPS};
    Repeat loop_accel{&await_loop_sid, ACCEL_LOOPS};

    PrintLastLoopDists print_last_six_loops{6, loop_start_sid};

    Sequence test_seq{
        // localize at whatever speed
        &localize,

        // continue at test speed to the loop
        &path_in_loop,

        // do three loops at top speed
        // first loop ignored for (accel / decel)
        // gets top speed data
        &test_speed,
        &loop_speed,

        // drop directly to crawl speed at the loop sensor
        // loop contains decel (v_f -> v_c) then cruise at v_c
        // gets deceleration data
        &crawl_speed,
        &loop_decel,

        // go to test speed at the loop sensor
        // loop contains accel (v_c -> v_f) then cruise at v_f (if done accel)
        &test_speed_2,
        &loop_accel,

        // print raw data from test loops,
        // &print_last_six_loops,
        &done_speed,
    };

    Invert invert_done_speed{&done_speed};
    Fallback tree{
        &test_seq,
        &invert_done_speed,
    };

    CalibrateTree(uint16_t speed          = DEFAULT_SPEED,
                  uint16_t loop_start_sid = DEFAULT_LOOP_SID)
        : cal_speed(speed), loop_start_sid(loop_start_sid) {}

    NodeResult tick(Blackboard &bb) override {
      auto res = tree.tick(bb);
      if (res != NodeResult::Success) {
        return res;
      }

      print_dists(bb.txs_tid, bb.dists);
      // once we've succeeded, we can calculate the speed, accel, and decel

      auto cursor_rev_it = std::ranges::find_if(
          std::views::reverse(bb.dists), [&, count = 0](auto const &x) mutable {
            return x.from_sid == loop_start_sid && ++count == TOTAL_LOOPS;
          });
      if (cursor_rev_it == std::views::reverse(bb.dists).end()) {
        bb.error_msg = "Failed to locate calibration loop start";
        return NodeResult::Failure;
      }

      auto cursor_it = std::prev(cursor_rev_it.base());

      // --- top speed: steady-state loops, average time first (sum d / sum t)
      int64_t speed_d_um = 0;
      int64_t speed_t    = 0;

      auto count = 0;
      Debug_Puts(bb.txs_tid, "from,to,dist(mm),ticks,mode,cal_speed");
      for (; cursor_it != bb.dists.end(); cursor_it++) {
        auto log = *cursor_it;
        if (log.from_sid == loop_start_sid && count++ == TOP_SPEED_LOOPS) {
          break;
        }
        speed_d_um += log.dx_um;
        speed_t    += log.d_ticks;
        Debug_Puts(bb.txs_tid, log.from_sid, ",", log.to_sid, ",",
                   log.dx_um / 1000, ",", log.d_ticks, ",speed,", cal_speed);
      }

      // --- decel: loop starts with a drop v_f -> v_c at the loop sensor,
      // then cruises at v_c for the rest of the loop
      int64_t decel_d_um = 0;
      int64_t decel_t    = 0;

      count = 0;
      for (; cursor_it != bb.dists.end(); cursor_it++) {
        auto log = *cursor_it;
        if (log.from_sid == loop_start_sid && count++ == DECEL_LOOPS) {
          break;
        }
        decel_d_um += log.dx_um;
        decel_t    += log.d_ticks;
        Debug_Puts(bb.txs_tid, log.from_sid, ",", log.to_sid, ",",
                   log.dx_um / 1000, ",", log.d_ticks, ",decel,", cal_speed);
      }

      // --- accel: only the first loop after the v_c -> v_f speed-up carries
      // the transient; later loops are pure cruise and only amplify v_f error
      int64_t accel_d_um = 0;
      int64_t accel_t    = 0;

      count = 0;
      for (; cursor_it != bb.dists.end(); cursor_it++) {
        auto log = *cursor_it;
        if (log.from_sid == loop_start_sid && count++ == ACCEL_LOOPS) {
          break;
        }
        accel_d_um += log.dx_um;
        accel_t    += log.d_ticks;
        Debug_Puts(bb.txs_tid, log.from_sid, ",", log.to_sid, ",",
                   log.dx_um / 1000, ",", log.d_ticks, ",accel,", cal_speed);
      }

      // --- solve the constant-accel model
      // v_f from this run's steady loops; v_c from the crawl-speed table
      if (speed_t == 0 || decel_t == 0 || accel_t == 0) {
        bb.error_msg = "Calibration collected no data";
        return NodeResult::Failure;
      }

      int64_t vf = speed_d_um / speed_t; // um/tick
      int64_t vc = bb.loco->v_max_umpt[SLOW_SPEED];

      bb.loco->v_max_umpt[cal_speed] = vf;

      // accel: d = vf*t - (vf-vc)^2 / (2a)  =>  a = (vf-vc)^2 / (2(vf*t - d))
      // scenario 2 (accel not done by loop end, d/t < (vc+vf)/2):
      //   v_avg = d/t, v_reached = 2*v_avg - vc  =>  a = 2*(v_avg - vc) / t
      // result in nm/tick^2 (x1000)
      if (2 * accel_d_um < (vc + vf) * accel_t) {
        bb.loco->a_nmpt2[cal_speed] =
            (2 * (accel_d_um - vc * accel_t) * 1000) / (accel_t * accel_t);
      } else if (vf * accel_t > accel_d_um) {
        bb.loco->a_nmpt2[cal_speed] =
            ((vf - vc) * (vf - vc) * 1000) / (2 * (vf * accel_t - accel_d_um));
      } // else: d == vf*t exactly, accel effectively instant; keep old value

      // decel: d = vc*t + (vf-vc)^2 / (2*dec) => dec = (vf-vc)^2 / (2(d -
      // vc*t))
      if (decel_d_um > vc * decel_t) {
        bb.loco->d_nmpt2[cal_speed] =
            ((vf - vc) * (vf - vc) * 1000) / (2 * (decel_d_um - vc * decel_t));
      }

      Debug_Puts(bb.txs_tid, "cal ", cal_speed, ": v_max ", vf, " um/t, a ",
                 bb.loco->a_nmpt2[cal_speed], " nm/t^2, d ",
                 bb.loco->d_nmpt2[cal_speed], " nm/t^2");

      return NodeResult::Success;
    }
  };

  struct CalibrateTrain : public LeafNode {
    uint16_t cal_speed  = 0;
    uint16_t curr_speed = 2;
    std::optional<CalibrateTree> train_cal{std::in_place, curr_speed};

    CalibrateTrain(uint16_t cal_speed) : cal_speed(cal_speed) {}

    NodeResult tick(Blackboard &bb) override {

      // cal_speed = 0 -> iterate through all speeds
      if (cal_speed == 0) {
        auto res = train_cal->tick(bb);

        // if this speed cal is done and we have more, update to next cal
        if (res == NodeResult::Success && curr_speed < 14) {
          curr_speed += 1;
          train_cal.emplace(curr_speed);
          return NodeResult::Running;
        }

        return res;
      } else {
        if (curr_speed != cal_speed) {
          curr_speed = cal_speed;
          train_cal.emplace(curr_speed);
        }
        return train_cal->tick(bb);
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

  struct NavigateTree : public Sequence {

    LocalizerTree localizer_tree{};
    PathToNode path_to_goal{sid('D', 4) - 1};
    Repeat path_to_goal_once{&path_to_goal, 1};
    SetSpeed max_speed{7};
    StopAtDonePath stop_at_done{};
    DebugPrintDists debug_print_dists{};

    NavigateTree(int goal_idx, uint16_t speed, int offset_mm)
        : path_to_goal{goal_idx}, max_speed{speed}, stop_at_done{offset_mm} {
      children.push(&localizer_tree);
      children.push(&path_to_goal_once);
      children.push(&max_speed);
      children.push(&stop_at_done);
      children.push(&debug_print_dists);
    }
  };

  struct ForeverNavigateTree : public TreeNode {
    Unif prng{time_get(), 0, TRACK_MAX - 1};

    Sequence seq{};
    LocalizerTree localizer_tree{};
    std::optional<PathToNode> path_to_goal{std::in_place,
                                           static_cast<int>(prng.nextNum())};
    SetSpeed max_speed{14};
    StopAtDonePath stop_at_done{};

    ForeverNavigateTree() {
      seq.children.push(&localizer_tree);
      seq.children.push(&(*path_to_goal));
      seq.children.push(&max_speed);
      seq.children.push(&stop_at_done);
    }

    NodeResult tick(Blackboard &bb) override {
      auto res = seq.tick(bb);
      if (res == NodeResult::Success) {
        path_to_goal.emplace(static_cast<int>(prng.nextNum()));
        max_speed    = SetSpeed{7};
        stop_at_done = StopAtDonePath{};
        Debug_Puts(bb.txs_tid, "Going to new node ",
                   bb.track[path_to_goal->goal_idx].name);
        return NodeResult::Running;
      }
      return res;
    }
  };
} // namespace

using Tree = std::variant<CalibrateTrain, PrintTrainStats, StopMeasureTree,
                          NavigateTree, ForeverNavigateTree, ReverseTree>;

void run_tree() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  auto tx_tid  = WhoIs(UART_TX_Server::NAME);

  Tree tree = PrintTrainStats{};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .track{TRACK_LAYOUT},
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
        Overloaded{[&](const TC::Tree::Init &msg) {
                     bb.state   = msg.state;
                     bb.loco_id = msg.loco_id;
                     bb.loco    = bb.state.get_loco(bb.loco_id);

                     switch (msg.tree_type) {
                     case TC::Tree::Type::CALIBRATE:
                       tree.emplace<CalibrateTrain>(msg.value1);
                       break;
                     case TC::Tree::Type::PRINT_TRAIN_STATS:
                       tree.emplace<PrintTrainStats>();
                       break;
                     case TC::Tree::Type::STOP_MEASURE:
                       tree.emplace<StopMeasureTree>(msg.value1);
                       break;
                     case TC::Tree::Type::REVERSE:
                       tree.emplace<ReverseTree>();
                       break;
                     case TC::Tree::Type::NAVIGATE: {
                       tree.emplace<NavigateTree>(msg.value1, msg.value2,
                                                  msg.value3);
                       break;
                     }
                     case TC::Tree::Type::FOREVER_NAVIGATE: {
                       tree.emplace<ForeverNavigateTree>();
                       break;
                     }
                     default: {
                       bb.error_msg = "Unknown tree type";
                       return false;
                     }
                     }
                     return true;
                   },
                   [&](const TC::Tree::Update &msg) {
                     bb.state.update_from_mrk(msg.mrk, msg.time);
                     bb.new_event = msg.mrk;
                     bb.curr_tick = msg.time;
                     return true;
                   },
                   [&](const TC::Tree::TrackReserved &msg) {
                     for (auto &node : msg.path) {
                       bb.track.reserve(node.node_idx, node.dir, msg.loco_id);
                     }
                     return true;
                   }},
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
      Debug_Puts(bb.txs_tid, "Done Tree");
      break;
    }
  }
  std::ignore = send<TC::Ack>(tcs_tid, TC::Tree::Exit{});
}