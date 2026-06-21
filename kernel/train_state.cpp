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
#define TIMING_ROW (SWITCH_ROW + 8)

void State::update_from_mrk(const MRKCmd &cmd) {
  std::visit(
      [&](const auto &command) {
        using Command = std::decay_t<decltype(command)>;

        if constexpr (std::is_same_v<Command, LightCmd>) {
          trains_dirty = true;
          for (Train &train : trains) {
            if (train.loco_id == command.loco_id) {
              train.light_on = command.value;
              return;
            }
          }
        } else if constexpr (std::is_same_v<Command, SpeedCmd>) {
          trains_dirty = true;
          for (Train &train : trains) {
            if (train.loco_id == command.loco_id) {
              train.requested_speed = command.speed;
              return;
            }
          }
        } else if constexpr (std::is_same_v<Command, DirectionCmd>) {
          trains_dirty = true;
          for (Train &train : trains) {
            if (train.loco_id == command.loco_id) {
              train.backward = command.backward;
              return;
            }
          }
        } else if constexpr (std::is_same_v<Command, SwitchCmd>) {
          switches_dirty = true;
          if (State::is_switch_id(command.sw_id)) {
            set_switch(command.sw_id, command.straight);
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
            for (Train &train : trains) {
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

void apply_state(const State &state) {
  // stop all trains first
  mcp2515_send(ControlCmd(ControlCmd::CMD_HALT).to_frame());

  for (const Train &train : state.trains) {
    mcp2515_send(LightCmd(train.loco_id, train.light_on).to_frame(),
                 train.loco_id * 1'000);
    mcp2515_send(SpeedCmd(train.loco_id, train.requested_speed).to_frame(),
                 train.loco_id * 1'000);
    mcp2515_send(DirectionCmd(train.loco_id, train.backward).to_frame(),
                 train.loco_id * 1'000);
  }

  for (int sw_id = 0; sw_id < 22; ++sw_id) {
    mcp2515_send(SwitchCmd(State::switch_id(sw_id),
                           state.is_switch_straight(State::switch_id(sw_id)))
                     .to_frame(),
                 sw_id * 1'000);
  }

  mcp2515_send(
      ControlCmd(state.stopped ? ControlCmd::CMD_STOP : ControlCmd::CMD_GO)
          .to_frame());
}