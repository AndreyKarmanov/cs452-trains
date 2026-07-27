#include "train_state.h"
#include "mrk.h"
#include "overloaded.h"

#define STATE_ROW "6"
#define STATE_ROW_INT 7
#define STATUS_ROW (STATE_ROW_INT + 1)
#define TRAIN_ROW (STATE_ROW_INT + 3)
#define SENSOR_ROW (TRAIN_ROW + MAX_TRAINS + 2)
#define SWITCH_ROW (SENSOR_ROW + 3)

std::optional<PathLocation> locate_train(const Track &track,
                                         const TrainState &train) {
  return train.e_path.decode(track).locate_at(train.d_um);
}

void TrackState::update(const MRKCmd &cmd) {
  std::visit(Overloaded{
                 [&](const LightCmd &cmd) {
                   for (TrainState &train : trains) {
                     if (train.id == cmd.loco_id) {
                       train.light_on = cmd.value;
                       return;
                     }
                   }
                 },
                 [&](const FunctionCmd &) {},
                 [&](const SpeedCmd &cmd) {
                   for (TrainState &train : trains) {
                     if (train.id == cmd.loco_id) {
                       train.req_speed = mrk_level_to_user_speed(cmd.speed);
                       return;
                     }
                   }
                 },
                 [&](const DirectionCmd &cmd) {
                   for (TrainState &train : trains) {
                     if (train.id == cmd.loco_id) {
                       train.backward = cmd.backward;
                       return;
                     }
                   }
                 },
                 [&](const SwitchCmd &cmd) {
                   if (TrackState::is_switch_id(cmd.sw_id)) {
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
                       stopped = false;
                     }
                     break;
                   case ControlCmd::CMD_STOP:
                     if (!stopped) {
                       stopped = true;
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
