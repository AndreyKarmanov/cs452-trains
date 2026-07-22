#include "train_state.h"
#include "mrk.h"
#include "overloaded.h"

#define STATE_ROW "6"
#define STATE_ROW_INT 7
#define STATUS_ROW (STATE_ROW_INT + 1)
#define TRAIN_ROW (STATE_ROW_INT + 3)
#define SENSOR_ROW (TRAIN_ROW + MAX_TRAINS + 2)
#define SWITCH_ROW (SENSOR_ROW + 3)

// Build path as if last_sensor node were prepended to e_path.
static Path effective_path(const Track &track, const TrainState &train) {
  Path path = train.e_path.decode(track);

  if (!train.last_sensor.has_value()) {
    return path;
  }

  int sensor_idx = train.last_sensor->sens.sid - 1;
  auto first     = path.peek();
  if (first.has_value() && first->node_idx == sensor_idx) {
    return path;
  }

  if (!first.has_value()) {
    const track_node &sensor = track[sensor_idx];
    path.push({.node_idx  = sensor_idx,
               .type      = sensor.type,
               .dx_next   = 0,
               .br_curved = false});
    return path;
  }

  auto prefix = track.find_path(sensor_idx, first->node_idx);
  if (!prefix.has_value()) {
    return path;
  }

  // operator+ mutates the left-hand path in place; local copy only.
  return prefix.value() + path;
}

std::optional<PathLocation> locate_train(const Track &track,
                                         const TrainState &train) {
  return effective_path(track, train).locate_at(train.d_um);
}

void State::update(const MRKCmd &cmd) {
  std::visit(Overloaded{
                 [&](const LightCmd &cmd) {
                   trains_dirty = true;
                   for (TrainState &train : trains) {
                     if (train.id == cmd.loco_id) {
                       train.light_on = cmd.value;
                       return;
                     }
                   }
                 },
                 [&](const FunctionCmd &) {},
                 [&](const SpeedCmd &cmd) {
                   trains_dirty = true;
                   for (TrainState &train : trains) {
                     if (train.id == cmd.loco_id) {
                       train.req_speed = mrk_level_to_user_speed(cmd.speed);
                       return;
                     }
                   }
                 },
                 [&](const DirectionCmd &cmd) {
                   trains_dirty = true;
                   for (TrainState &train : trains) {
                     if (train.id == cmd.loco_id) {
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
                         sensors.peek_last() != cmd.sid) {
                       sensors_dirty = true;
                       if (sensors.size() == MAX_SENSORS_RECENT) {
                         sensors.pop();
                       }
                       sensors.push(cmd.sid);
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
