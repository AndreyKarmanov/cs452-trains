#include "train_state.h"
#include "mrk.h"
#include "overloaded.h"

#define STATE_ROW "6"
#define STATE_ROW_INT 7
#define STATUS_ROW (STATE_ROW_INT + 1)
#define TRAIN_ROW (STATE_ROW_INT + 3)
#define SENSOR_ROW (TRAIN_ROW + MAX_TRAINS + 2)
#define SWITCH_ROW (SENSOR_ROW + 3)

void State::update_from_mrk(const MRKCmd &cmd, uint32_t tick) {
  (void)tick;
  std::visit(Overloaded{
                 [&](const LightCmd &cmd) {
                   trains_dirty = true;
                   for (TrainState &train : trains) {
                     if (train.loco_id == cmd.loco_id) {
                       train.light_on = cmd.value;
                       return;
                     }
                   }
                 },
                 [&](const SpeedCmd &cmd) {
                   trains_dirty = true;
                   for (TrainState &train : trains) {
                     if (train.loco_id == cmd.loco_id) {
                       train.req_speed = mrk_level_to_user_speed(cmd.speed);
                       return;
                     }
                   }
                 },
                 [&](const DirectionCmd &cmd) {
                   trains_dirty = true;
                   for (TrainState &train : trains) {
                     if (train.loco_id == cmd.loco_id) {
                       train.backward = cmd.backward;
                       return;
                     }
                   }
                 },
                 [&](const SwitchCmd &cmd) {
                   switches_dirty = true;
                   if (State::is_switch_id(cmd.sw_id)) {
                     // special case for sw 153/154 and 155/156
                     // if 153 is curved, 154 must be straight, and v.v., same
                     // for 155/156
                     set_switch(cmd.sw_id, cmd.straight);
                     if (cmd.sw_id == 153) {
                       set_switch(154, true);
                     } else if (cmd.sw_id == 154) {
                       set_switch(153, true);
                     } else if (cmd.sw_id == 155) {
                       set_switch(156, true);
                     } else if (cmd.sw_id == 156) {
                       set_switch(155, true);
                     }
                   }
                 },
                 [&](const SensorData &cmd) {
                   if (cmd.new_state) {
                     if (sensors.size() == 0 ||
                         sensors.peek_last() != cmd.sensor_id) {
                       sensors_dirty = true;
                       if (sensors.size() == MAX_SENSORS_RECENT) {
                         sensors.pop();
                       }
                       sensors.push(cmd.sensor_id);
                     }
                   }
                 },
                 [&](const ControlCmd &cmd) {
                   switch (cmd.type) {
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
                 },
                 [&](const UnknownCmd &) { return; },
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
