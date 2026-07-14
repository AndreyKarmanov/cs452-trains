#include "train_state.h"
#include "mrk.h"
#include "pathfind.h"
#include "track_node.h"
#include <type_traits>

#define STATE_ROW "6"
#define STATE_ROW_INT 7
#define STATUS_ROW (STATE_ROW_INT + 1)
#define TRAIN_ROW (STATE_ROW_INT + 3)
#define SENSOR_ROW (TRAIN_ROW + MAX_TRAINS + 2)
#define SWITCH_ROW (SENSOR_ROW + 3)

void State::update_from_mrk(const MRKCmd &cmd, uint32_t tick) {
  (void)tick;
  std::visit(
      [&](const auto &command) {
        using Command = std::decay_t<decltype(command)>;

        if constexpr (std::is_same_v<Command, LightCmd>) {
          trains_dirty = true;
          for (TrainState &train : trains) {
            if (train.loco_id == command.loco_id) {
              train.light_on = command.value;
              return;
            }
          }
        } else if constexpr (std::is_same_v<Command, SpeedCmd>) {
          trains_dirty = true;
          for (TrainState &train : trains) {
            if (train.loco_id == command.loco_id) {
              train.req_speed = mrk_level_to_user_speed(command.speed);
              return;
            }
          }
        } else if constexpr (std::is_same_v<Command, DirectionCmd>) {
          trains_dirty = true;
          for (TrainState &train : trains) {
            if (train.loco_id == command.loco_id) {
              train.backward = command.backward;
              return;
            }
          }
        } else if constexpr (std::is_same_v<Command, SwitchCmd>) {
          switches_dirty = true;
          if (State::is_switch_id(command.sw_id)) {
            // special case for sw 153/154 and 155/156
            // if 153 is curved, 154 must be straight, and v.v., same for
            // 155/156
            set_switch(command.sw_id, command.straight);
            if (command.sw_id == 153) {
              set_switch(154, true);
            } else if (command.sw_id == 154) {
              set_switch(153, true);
            } else if (command.sw_id == 155) {
              set_switch(156, true);
            } else if (command.sw_id == 156) {
              set_switch(155, true);
            }
          }
        } else if constexpr (std::is_same_v<Command, SensorData>) {
          if (command.new_state) {
            if (sensors.size() == 0 ||
                sensors.peek_last() != command.sensor_id) {
              sensors_dirty = true;
              if (sensors.size() == MAX_SENSORS_RECENT) {
                sensors.pop();
              }
              sensors.push(command.sensor_id);
            }
          }
        } else if constexpr (std::is_same_v<Command, ControlCmd>) {
          switch (command.type) {
          case ControlCmd::CMD_GO:
            if (stopped) {
              stopped      = false;
              status_dirty = true;
            }
            break;
          case ControlCmd::CMD_STOP:
            if (!stopped) {
              stopped      = true;
              status_dirty = true;
            }
            break;
          case ControlCmd::CMD_HALT:
            for (TrainState &train : trains) {
              train.req_speed = 0;
            }
            break;
          default:
            break;
          }
        }
      },
      cmd);
}

static uint64_t isqrt(uint64_t n) {
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

static const track_edge *next_edge(const State &state, const track_node &node) {
  switch (node.type) {
  case NODE_SENSOR:
  case NODE_MERGE:
  case NODE_ENTER:
    return node.edge[DIR_AHEAD].dest ? &node.edge[DIR_AHEAD] : nullptr;
  case NODE_BRANCH:
    return state.is_switch_straight(node.num) ? &node.edge[DIR_STRAIGHT]
                                              : &node.edge[DIR_CURVED];
  default:
    return nullptr;
  }
}

int State::get_next_sensor_predictions(track_node *current_node,
                                       const TrainState &loco,
                                       SensorPrediction *result, int length,
                                       node_type filter_node_type) {
  if (!current_node || !result || length <= 0) {
    return 0;
  }

  int count              = 0;
  int travelled          = 0;
  const track_node *node = current_node;

  for (int steps = 0; steps < TRACK_MAX && count < length; ++steps) {
    const track_edge *edge = next_edge(*this, *node);
    if (!edge) {
      break;
    }

    travelled += edge->dist;
    node       = edge->dest;

    if (filter_node_type != NODE_NONE && node->type != filter_node_type) {
      continue;
    }
    if (node == current_node) {
      continue;
    }

    const int dist_um    = travelled * 1000;
    const int base_ticks = predict_ticks(dist_um, loco.ve_nm, loco);
    result[count]        = {
        .sensor_id = static_cast<uint16_t>((node - Pathfind::track) + 1),
        .min_trigger_ticks  = static_cast<uint32_t>(base_ticks * 4 / 5),
        .max_trigger_ticks  = static_cast<uint32_t>(base_ticks * 6 / 5),
        .did_error          = (count == 1),
        .predicted_tick     = static_cast<uint32_t>(base_ticks),
        .v_at_prediction_nm = loco.ve_nm,
    };
    ++count;
  }
  return count;
}
