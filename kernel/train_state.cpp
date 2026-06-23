#include "train_state.h"
#include "mcp2515.h"
#include "mrk.h"
#include <type_traits>

#define STATE_ROW "6"
#define STATE_ROW_INT 7
#define STATUS_ROW (STATE_ROW_INT + 1)
#define TRAIN_ROW (STATE_ROW_INT + 3)
#define SENSOR_ROW (TRAIN_ROW + MAX_TRAINS + 2)
#define SWITCH_ROW (SENSOR_ROW + 3)

void State::update_from_mrk(const MRKCmd &cmd) {
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
              train.requested_speed = command.speed;
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
              train.requested_speed = 0;
            }
            break;
          default:
            break;
          }
        }
      },
      cmd);
}