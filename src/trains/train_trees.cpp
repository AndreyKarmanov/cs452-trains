#include "train_trees.h"
#include "behaviour_tree.h"
#include "message.h"
#include "mrk.h"
#include "overloaded.h"
#include "pathfind.h"
#include "rng.h"
#include "time.h"
#include "track_data.h"
#include "track_node.h"
#include "train_control.h"
#include <algorithm>
#include <cstdint>
#include <iterator>
#include <numeric>
#include <optional>
#include <ranges>

namespace {
  auto sid = [](char b, int n) -> uint16_t { return (b - 'A') * 16 + n; };
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
      Debug_Puts(txs_tid, dist.from.to_string(), ", ", dist.to.to_string(),
                 ", ", dist.d_um / 1000, ", ", dist.d_t, ", ",
                 dist.d_um / dist.d_t, ", ", ticks);
      ticks += dist.d_t;
    }
  }

  struct DebugPrintPath : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      StaticString<128> path_str{};
      path_str.append("Path: ", bb.path.size(), " ");
      for (const auto &node : bb.path) {
        path_str.append(bb.track[node.node_idx].name,
                        node.type == NODE_BRANCH ? node.br_curved ? "C " : "S "
                                                 : " ",
                        " >");
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
      Debug_Puts(bb.txs_tid, "Train ", bb.loco->id, " Stats: ");
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
        if ((*it).to.sid == loop_start_sid) {
          if (found_start == false) {
            found_start = true;
          } else {
            loops_to_use--;
          }
        }
        if (found_start && loops_to_use > 0) {
          dist_um += (*it).d_um;
          ticks   += (*it).d_t;
          sensors++;
          Debug_Puts(bb.txs_tid, (*it).from.sid, ", ", (*it).to.sid, ", ",
                     (*it).d_um / 1000, ", ", (*it).d_t);
        }
      }

      Debug_Puts(bb.txs_tid, "dist(mm), ticks, speed(um/t), sensors\n\r",
                 dist_um / 1000, ticks, dist_um / ticks, sensors);
      return NodeResult::Success;
    }
  };

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
            bb.tcs_tid, TC::Cmd::Speed{.id = bb.loco->id, .speed = req_speed});
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
    bool done{false};
    bool backward{false};
    bool sent_cmd{false};
    SetDirectionNode(bool backward) : backward(backward) {}
    NodeResult tick(Blackboard &bb) override {

      if (done || (sent_cmd && bb.loco->backward == backward)) {
        done = true;
        return NodeResult::Success;
      }

      auto resp =
          send<TC::Ack>(bb.tcs_tid, TC::Cmd::Direction{.id       = bb.loco->id,
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
    int sid           = -1;
    AwaitSensorNode() = default;
    AwaitSensorNode(int sid) : sid(sid) {}
    NodeResult tick(Blackboard &bb) override {
      if (auto sens = std::get_if<SensorData>(&bb.new_event);
          sens && sens->new_state == 1 && (sens->sid == sid || sid == -1)) {
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

      uint64_t v_m_nm = bb.loco->v_max_umpt[bb.loco->req_speed] * 1000;
      uint64_t v_i_nm = bb.loco->ve_nm;
      uint64_t d_t    = (bb.curr_tick - last_tick) * (TICK_TIME_US / 1'000);
      last_tick       = bb.curr_tick;

      uint64_t delta_um = 0;
      if (v_m_nm >= v_i_nm) {
        // accelerating (or cruising): constant a until v_max, then cruise
        uint64_t a     = bb.loco->a_nmpt2[bb.loco->req_speed];
        uint64_t t_a   = std::min((v_m_nm - v_i_nm) / a, d_t);
        bb.loco->ve_nm = v_i_nm + a * t_a;

        uint64_t t_c = d_t - t_a;
        delta_um = ((a * t_a * t_a) / 2 + v_m_nm * t_c + v_i_nm * t_a) / 1000;

        // track which speed level our current velocity corresponds to,
        // so a later slow-down uses the right decel constant
        decel_from_speed = bb.loco->req_speed;
      } else {
        uint64_t d     = bb.loco->d_nmpt2[decel_from_speed];
        uint64_t t_d   = std::min((v_i_nm - v_m_nm) / d, d_t);
        bb.loco->ve_nm = v_i_nm - d * t_d;

        uint64_t t_c = d_t - t_d;
        delta_um = (v_i_nm * t_d - (d * t_d * t_d) / 2 + v_m_nm * t_c) / 1000;
      }

      // if the train is reversed, we are backing up
      bb.loco->d_um += delta_um;

      auto ve_um = bb.loco->ve_nm / (1'000);
      if (ve_um == 0) {
        bb.loco->stop_dist_um = 0;
      } else {
        auto &p               = bb.loco->stop_params;
        bb.loco->stop_dist_um = p.c0 + p.c1 * ve_um + p.c2 * ve_um * ve_um;
        if (bb.loco->backward) {
          bb.loco->stop_dist_um += 100'000;
        }
      }
      return NodeResult::Success;
    }
  };

  struct AttributeSensorNode : public LeafNode {
    static constexpr int PCT_TOLERANCE = 30;

    NodeResult tick(Blackboard &bb) override {
      if (auto sens = std::get_if<SensorData>(&bb.new_event);
          sens && sens->new_state == 1) {

        // auto attribute if no inital node
        if (bb.loco->inital_node_idx == -1) {
          bb.loco->last_sensor.emplace(
              TrainState::SeenSensor{*sens, bb.curr_tick});

          return NodeResult::Success;
        }

        // filter for inital node
        if (!bb.loco->last_sensor.has_value()) {
          if (bb.loco->inital_node_idx + 1 != sens->sid) {
            return NodeResult::Running;
          }
          bb.loco->last_sensor.emplace(
              TrainState::SeenSensor{*sens, bb.curr_tick});
          return NodeResult::Success;
        }

        if (!bb.path.empty()) {
          // check if the sensor is in the path, an dis reserved
          auto idx = std::ranges::find(
              bb.path, sens->sid, [&, dist = 0](PathNode &node) mutable {
                if (node.type == NODE_SENSOR && dist <= bb.loco->res_dist_um) {
                  return node.node_idx + 1;
                }
                dist += node.dx_next * 1000;
                return -1;
              });

          if (idx != bb.path.end()) {
            Debug_Puts(bb.txs_tid, sens->to_string(),
                       " Attributed: ", bb.loco->id, " in reserved path");
            bb.loco->last_sensor.emplace(
                TrainState::SeenSensor{*sens, bb.curr_tick});
            return NodeResult::Success;
          }
        }

        auto start_node_idx = bb.path.empty()
                                  ? bb.loco->last_sensor->sens.sid - 1
                                  : bb.path.peek()->node_idx;
        auto path           = bb.track.find_path(start_node_idx, sens->sid - 1);

        // if there's no path, or 20% off our estimate, we ignore
        if (!path.has_value()) {
          Debug_Puts(bb.txs_tid, sens->to_string(), " No Path: ", bb.loco->id);
          return NodeResult::Running;
        }

        auto sens_dist_um = path->dist_mm * 1000;
        auto abs_d_um     = bb.loco->d_um < 0 ? -bb.loco->d_um : bb.loco->d_um;
        if (sens_dist_um > (abs_d_um * (100 + PCT_TOLERANCE)) / 100 ||
            sens_dist_um < (abs_d_um * (100 - PCT_TOLERANCE)) / 100) {
          Debug_Puts(bb.txs_tid, sens->to_string(),
                     "Out of Range: ", bb.loco->id, " ", sens->to_string(),
                     " pos ", bb.loco->d_um * 100 / sens_dist_um, "% ",
                     (bb.loco->d_um - sens_dist_um) / 1000, " mm");
          return NodeResult::Running;
        }

        Debug_Puts(bb.txs_tid, sens->to_string(), "Attributed: ", bb.loco->id,
                   " ", sens->sid, " ", sens->to_string(), " pos ",
                   bb.loco->d_um * 100 / sens_dist_um, "% ",
                   (bb.loco->d_um - sens_dist_um) / 1000, " mm");
        bb.loco->last_sensor.emplace(
            TrainState::SeenSensor{*sens, bb.curr_tick});
      }

      return NodeResult::Success;
    }
  };

  struct LocalizerNode : public LeafNode {

    NodeResult tick(Blackboard &bb) override {
      if (auto sens = std::get_if<SensorData>(&bb.new_event);
          sens && sens->new_state == 1) {

        if (bb.path.empty()) {
          bb.loco->d_um = 0;
          return NodeResult::Success;
        }

        // try to find the sensor in our path. If not there, repath.
        auto idx = std::ranges::find(bb.path, sens->sid, [](PathNode &node) {
          if (node.type == NODE_SENSOR) {
            return node.node_idx + 1;
          }
          return -1;
        });
        if (idx == bb.path.end()) {
          Debug_Puts(bb.txs_tid, "Repathing from ", sens->sid,
                     sens->to_string(), " to ",
                     bb.track[(*(bb.path.end() - 1)).node_idx].name);
          bb.path.clear();
        } else {
          auto d_mm = std::accumulate(
              bb.path.begin(), idx, 0,
              [](int acc, const PathNode &node) { return acc + node.dx_next; });
          bb.loco->res_dist_um =
              std::max(bb.loco->res_dist_um - d_mm * 1000, 0);

          if (bb.loco->last_sensor.has_value()) {
            if (bb.dists.size() == bb.dists.capacity()) {
              bb.dists.pop();
            }
            bb.dists.push({
                .from = bb.loco->last_sensor->sens,
                .to   = *sens,
                .d_um = d_mm * 1000,
                .d_t  = bb.curr_tick - bb.loco->last_sensor->tick,
            });
          }
          auto skipped_nodes = std::distance(bb.path.begin(), idx) + 1;
          bb.path.pop(skipped_nodes - 1);
        }
        bb.loco->e_path = bb.path;
        bb.loco->d_um   = 0;
      }
      return NodeResult::Success;
    }
  };

  struct PathReservationNode : public LeafNode {
    static constexpr int STOP_DIST_BUF_PCT = 20;
    static constexpr int EXTRA_BUFFER_UM   = 300'000; // one train length
    int last_print_tick{0};
    int last_res_dist_um{0};
    int stopped_since_tick{0};

    SetSpeed stop{0};
    bool reservation_stop{false};
    SetDirectionNode dir_node{false};
    SetSpeed go{8};
    PathReservationNode(uint16_t go_speed = 8) : go(go_speed) {}

    NodeResult tick(Blackboard &bb) override {
      int64_t res_dist_um = 0;
      bool fully_reserved{true};

      int stop_buf_um =
          (bb.loco->stop_dist_um * (100 + STOP_DIST_BUF_PCT)) / 100 +
          EXTRA_BUFFER_UM;

      int lookahead_um = stop_buf_um + bb.loco->d_um +
                         (bb.loco->ve_nm * TICKS_PER_S * 2) / 1000;

      auto res = send<TC::Cmd::ReserveResponse>(
          bb.tcs_tid, TC::Cmd::Reserve{
                          .id           = bb.loco->id,
                          .lookahead_um = lookahead_um,
                          .path         = bb.path,
                      });
      bb.loco->res_dist_um = res->res_dist_um;
      res_dist_um          = bb.loco->res_dist_um;

      if (bb.curr_tick - last_print_tick > TICKS_PER_S) {
        Offset_Puts(
            bb.txs_tid, 30 + bb.loco->id, bb.loco->id,
            " Path dist: ", bb.path.dist_mm, " dx_um: ", bb.loco->d_um / 1000,
            " res dist: ", res_dist_um / 1000,
            " last_res_dist: ", last_res_dist_um / 1000,
            " stop_buf: ", stop_buf_um / 1000, " stopped: ", reservation_stop,
            " time stopped: ", bb.curr_tick - stopped_since_tick, "\033K[");

        last_print_tick = bb.curr_tick;
      }

      // if we're stopped, but we're reserving more
      // we shouldn't try to reverse so soon
      if (reservation_stop && last_res_dist_um < res_dist_um) {
        stopped_since_tick = bb.curr_tick;
      }

      if (fully_reserved) {
        // if fully reserved, we don't care about reserving more
        // reset speed if we stopped it
        if (reservation_stop) {
          reservation_stop = false;
          stop             = SetSpeed{0};
          return go.tick(bb);
        }
        return NodeResult::Success;
      } else if (!reservation_stop &&
                 res_dist_um <= stop_buf_um + bb.loco->d_um) {
        // if we're coming up on stopping distance, we stop.
        go                 = SetSpeed{go.req_speed};
        reservation_stop   = true;
        stopped_since_tick = bb.curr_tick;
        stop.tick(bb);
        return NodeResult::Running;
      } else if (reservation_stop && last_res_dist_um < res_dist_um) {
        // if we reserved more since last move, we can go again.
        reservation_stop = false;
        stop             = SetSpeed{0};
        return go.tick(bb);
      } else if (reservation_stop &&
                 stopped_since_tick + TICKS_PER_S * (5 + bb.loco->extra_delay) <
                     static_cast<int>(bb.curr_tick)) {
        // we are stopped, so we can flip directions ez
        // we try to navigate by finding a path starting at our latest reserved
        // if we can find a path, we take that!

        Debug_Puts(bb.txs_tid, bb.loco->id, " Deadlock reversing");

        auto last_reserved = bb.path.end();
        auto dist_reserved = 0;
        for (auto it = bb.path.begin(); it < bb.path.end(); it++) {
          auto node = *it;
          if (dist_reserved >= bb.loco->res_dist_um) {
            break;
          }
          dist_reserved += node.dx_next;
          last_reserved  = it;
        }

        if (last_reserved == bb.path.end()) {
          stopped_since_tick = bb.curr_tick;
          Debug_Puts(bb.txs_tid, bb.loco->id,
                     " Can't deadlock reverse, no reserved.");
          // we don't have any reserved
          // get scared and flip out
          stop.tick(bb);
          return NodeResult::Running;
        }

        auto unreserved_nodes = std::distance(last_reserved, bb.path.end() - 1);
        Puts(bb.web_tid, bb.loco->id, " Unres: ", unreserved_nodes,
             " Old Path: ", bb.path.to_string(&bb.track));
        for (int i = 0; i < unreserved_nodes; ++i) {
          bb.path.dist_mm -= bb.path.pop_back()->dx_next;
        }
        auto new_path_start = bb.path.reverse();

        auto start_node = new_path_start.peek_last();
        auto goal_node  = bb.path.peek_last();

        // we have something reserved.
        // we create a new path.
        auto new_path_opt =
            bb.track.find_path(start_node->node_idx, goal_node->node_idx);

        if (!new_path_opt.has_value()) {
          stop.tick(bb);
          return NodeResult::Running;
        }
        auto new_path = new_path_opt.value();

        auto c_path = new_path_start + new_path;
        Puts(bb.web_tid, bb.loco->id,
             " New Path: ", c_path.to_string(&bb.track));

        StaticString<128> path_str{};
        path_str.append(bb.loco->id, " Res Path: ");
        Puts(bb.web_tid, path_str);
        bb.loco->d_um   = bb.path.dist_mm * 1000 - bb.loco->d_um;
        c_path.track    = &bb.track;
        bb.path         = c_path;
        bb.loco->e_path = bb.path;
        dir_node        = SetDirectionNode{!bb.loco->backward};
        dir_node.tick(bb);
        Puts(bb.web_tid, bb.loco->id, " Completed direction flip");

        stopped_since_tick = bb.curr_tick;
        return NodeResult::Running;
      } else if (reservation_stop) {
        // otherwise, if we haven't reserved more, keep waiting
        stop.tick(bb);
        return NodeResult::Running;
      }

      last_res_dist_um = res_dist_um;

      return NodeResult::Success;
    }
  };

  struct PathLookaheadNode : public LeafNode {
    NodeResult tick(Blackboard &bb) override {
      int dist_um = 0;
      for (auto &node : bb.path) {
        if (dist_um >= bb.loco->res_dist_um) {
          break;
        }

        // switch if we have enough time for the switch to switch.
        if (node.type == NODE_BRANCH &&
            node.br_curved !=
                bb.state.is_switch_curved(bb.track[node.node_idx].num)) {
          if ((dist_um * 1000 / bb.loco->ve_nm) > TICKS_PER_S / 3) {
            auto res = send<TC::Ack>(
                bb.tcs_tid,
                TC::Cmd::Switch(bb.track[node.node_idx].num, !node.br_curved));
            if (!res.has_value()) {
              bb.error_msg = "Switch cmd failed";
              return NodeResult::Failure;
            }
          }
        }
        dist_um += node.dx_next * 1000;
      }
      return NodeResult::Success;
    }
  };

  struct StopAtDonePath : public LeafNode {

    SetSpeed stop{0};
    int offset_mm{0};
    bool stopping = false;

    int last_print_tick{0};
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
      int64_t remaining_um =
          1000 * std::ranges::fold_left(bb.path, 0,
                                        [](int acc, const PathNode &node) {
                                          return acc + node.dx_next;
                                        }) -
          bb.loco->d_um + offset_mm * 1000;

      if (remaining_um < bb.loco->stop_dist_um) {
        Debug_Puts(bb.txs_tid, "Stopping at done path: ", remaining_um, " < ",
                   bb.loco->stop_dist_um);
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
    SetDirectionNode dir{false};
    SetSpeed set_speed{0};

    NodeResult tick(Blackboard &bb) override {

      if (!initalized) {
        dir        = SetDirectionNode{!bb.loco->backward};
        set_speed  = SetSpeed{bb.loco->req_speed};
        initalized = true;
      }

      if (bb.loco->ve_nm / 1000 > 10) {
        auto res = stop_speed.tick(bb);
        if (res != NodeResult::Success) {
          return res;
        }
        return NodeResult::Running;
      }

      auto res = dir.tick(bb);
      if (res != NodeResult::Success) {

        return res;
      }

      if (set_speed.req_speed > 0) {
        res = set_speed.tick(bb);
        if (res != NodeResult::Success) {
          return res;
        }
      }

      return NodeResult::Success;
    }
  };

  struct PathToNode : public LeafNode {
    int goal_idx{};

    std::optional<ReverseTree> rev_tree{std::in_place};

    PathToNode(int goal_idx) : goal_idx(goal_idx) {}

    NodeResult tick(Blackboard &bb) override { return tick(bb, false); }

    NodeResult tick(Blackboard &bb, bool force_path = false) {
      if (!bb.loco->last_sensor.has_value() && bb.path.empty()) {
        bb.error_msg = "Failed to find start";
        return NodeResult::Failure;
      }

      auto start_idx  = bb.path.empty() ? bb.loco->last_sensor->sens.sid - 1
                                        : bb.path.peek_last()->node_idx;
      auto startr_idx = bb.track[start_idx].reverse->idx;
      auto goalr_idx  = bb.track[goal_idx].reverse->idx;

      if (auto last_node = bb.path.peek_last();
          last_node.has_value() &&
          (last_node->node_idx == goal_idx ||
           last_node->node_idx == goalr_idx) &&
          !force_path) {
        return NodeResult::Success;
      }

      bool should_reverse = false;

      // start to goal
      auto path_opt = bb.track.find_path(start_idx, goal_idx);

      // start to reverse goal
      if (!path_opt.has_value()) {
        path_opt = bb.track.find_path(start_idx, goalr_idx);
      }

      // reverse start to goal
      if (!path_opt.has_value()) {
        path_opt       = bb.track.find_path(startr_idx, goal_idx);
        should_reverse = true;
      }

      // reverse start to reverse goal
      if (!path_opt.has_value()) {
        path_opt       = bb.track.find_path(startr_idx, goalr_idx);
        should_reverse = true;
      }

      if (!path_opt.has_value()) {
        bb.error_msg = "No path to goal";
        return NodeResult::Failure;
      }

      auto new_path = path_opt.value();

      bb.path.track = &bb.track;

      if (should_reverse) {
        auto res = rev_tree->tick(bb);
        if (res == NodeResult::Success) {
          rev_tree.emplace();
          bb.loco->d_um = bb.path.dist_mm * 1000 - bb.loco->d_um;
          bb.path       = new_path;
        } else {
          return res;
        }
      } else {
        bb.path = bb.path + new_path;
      }

      bb.loco->e_path = bb.path;
      return NodeResult::Success;
    }
  };

  struct LocalizerTree : public LeafNode {
    UpdateModel model{};
    SetSpeed set_speed{CRAWL_SPEED};
    AttributeSensorNode attribute_sensor{};
    LocalizerNode localize{};
    PathReservationNode reserve{};
    PathLookaheadNode lookahead{};

    Sequence loop{
        &model, &set_speed, &attribute_sensor, &localize, &reserve, &lookahead,
    };

    NodeResult tick(Blackboard &bb) override {

      // if we're already moving, keep the same speed
      // otherwise sets to crawl speed to start localizing
      if (!bb.loco->last_sensor.has_value() && bb.loco->req_speed != 0 &&
          set_speed.req_speed != bb.loco->req_speed) {
        set_speed = SetSpeed{bb.loco->req_speed};
      }

      auto res = loop.tick(bb);
      if (res == NodeResult::Failure) {
        return NodeResult::Failure;
      } else if (!bb.loco->last_sensor.has_value()) {
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
            return x.from.sid == loop_start_sid && ++count == TOTAL_LOOPS;
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
        if (log.from.sid == loop_start_sid && count++ == TOP_SPEED_LOOPS) {
          break;
        }
        speed_d_um += log.d_um;
        speed_t    += log.d_t;
        Debug_Puts(bb.txs_tid, log.from.sid, ",", log.to.sid, ",",
                   log.d_um / 1000, ",", log.d_t, ",speed,", cal_speed);
      }

      // --- decel: loop starts with a drop v_f -> v_c at the loop sensor,
      // then cruises at v_c for the rest of the loop
      int64_t decel_d_um = 0;
      int64_t decel_t    = 0;

      count = 0;
      for (; cursor_it != bb.dists.end(); cursor_it++) {
        auto log = *cursor_it;
        if (log.from.sid == loop_start_sid && count++ == DECEL_LOOPS) {
          break;
        }
        decel_d_um += log.d_um;
        decel_t    += log.d_t;
        Debug_Puts(bb.txs_tid, log.from.sid, ",", log.to.sid, ",",
                   log.d_um / 1000, ",", log.d_t, ",decel,", cal_speed);
      }

      // --- accel: only the first loop after the v_c -> v_f speed-up carries
      // the transient; later loops are pure cruise and only amplify v_f error
      int64_t accel_d_um = 0;
      int64_t accel_t    = 0;

      count = 0;
      for (; cursor_it != bb.dists.end(); cursor_it++) {
        auto log = *cursor_it;
        if (log.from.sid == loop_start_sid && count++ == ACCEL_LOOPS) {
          break;
        }
        accel_d_um += log.d_um;
        accel_t    += log.d_t;
        Debug_Puts(bb.txs_tid, log.from.sid, ",", log.to.sid, ",",
                   log.d_um / 1000, ",", log.d_t, ",accel,", cal_speed);
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
    SetSpeed max_speed{7};
    StopAtDonePath stop_at_done{};
    DebugPrintDists debug_print_dists{};

    NavigateTree(int goal_idx, uint16_t speed, int offset_mm)
        : path_to_goal{goal_idx}, max_speed{speed}, stop_at_done{offset_mm} {
      children.push(&localizer_tree);
      children.push(&path_to_goal);
      children.push(&max_speed);
      children.push(&stop_at_done);
      children.push(&debug_print_dists);
    }
  };

  struct ForeverNavigateTree : public TreeNode {
    Unif prng{time_get(), 0, 79};
    bool random{true};

    Sequence seq{};
    LocalizerTree localizer_tree{};
    std::optional<PathToNode> path_to_goal{std::in_place,
                                           static_cast<int>(prng.nextNum())};
    SetSpeed max_speed{14};
    StopAtDonePath stop_at_done{};
    WaitNode wait{TICKS_PER_S};

    ForeverNavigateTree(uint16_t speed) : random{true}, max_speed{speed} {
      seq.children.push(&localizer_tree);
      seq.children.push(&(*path_to_goal));
      seq.children.push(&max_speed);
      seq.children.push(&stop_at_done);
    }

    ForeverNavigateTree(int goal_idx, uint16_t speed)
        : random{false}, path_to_goal{goal_idx}, max_speed{speed} {
      seq.children.push(&localizer_tree);
      seq.children.push(&(*path_to_goal));
      seq.children.push(&max_speed);
      seq.children.push(&stop_at_done);
    }

    NodeResult tick(Blackboard &bb) override {
      auto res = seq.tick(bb);
      if (res == NodeResult::Success) {
        if (random) {
          path_to_goal.emplace(static_cast<int>(prng.nextNum()));
        } else {
          path_to_goal->tick(bb, true);
        }
        max_speed    = SetSpeed{max_speed.req_speed};
        stop_at_done = StopAtDonePath{};
        wait         = WaitNode{TICKS_PER_S};
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
  auto web_tid = WhoIs(UART03_TX_Server::NAME);

  Tree tree = PrintTrainStats{};
  Blackboard bb{
      .tcs_tid = tcs_tid,
      .txs_tid = tx_tid,
      .web_tid = web_tid,
      .track{TrainControlServer<>::TRACK},
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
        Overloaded{
            [&](const TC::Tree::Init &msg) {
              bb.state = msg.state;
              bb.loco  = bb.state.get_loco(msg.loco_id);
              bb.path  = bb.loco->e_path.decode(bb.track);
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
                tree.emplace<NavigateTree>(msg.value1, msg.value2, msg.value3);
                break;
              }
              case TC::Tree::Type::FOREVER_NAVIGATE: {
                tree.emplace<ForeverNavigateTree>(msg.value1);
                break;
              }
              case TC::Tree::Type::LOOP_NODE:
                tree.emplace<ForeverNavigateTree>(msg.value1, msg.value2);
                break;
              default: {
                bb.error_msg = "Unknown tree type";
                return false;
              }
              }
              return true;
            },
            [&](const TC::Tree::Update &msg) {
              auto loco    = bb.loco;
              bb.state     = msg.state;
              bb.loco      = bb.state.get_loco(loco->id);
              bb.loco      = loco;
              bb.new_event = msg.mrk;
              bb.curr_tick = msg.time;
              return true;
            },
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
      Debug_Puts(bb.txs_tid, "Done Tree");
      break;
    }
  }
  std::ignore = send<TC::Ack>(tcs_tid, TC::Tree::Exit{});
}