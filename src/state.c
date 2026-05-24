#include "state.h"
#include "can.h"
#include "mcp2515.h"
#include "uart.h"
#include "time.h"

#define STATE_ROW "6"
#define STATE_ROW_INT 7
#define STATUS_ROW (STATE_ROW_INT + 1)
#define TRAIN_ROW (STATE_ROW_INT + 3)
#define SENSOR_ROW (TRAIN_ROW + MAX_TRAINS + 2)
#define SWITCH_ROW (SENSOR_ROW + 3)
#define TIMING_ROW (SWITCH_ROW + 8)

void State::update_from_mrk(const MRK_CMD& cmd) {
    
    uint8_t cmd_index = cmd.index();
    if (command_timings_start[cmd_index]) {
        timings_dirty = true;
        command_timings[cmd_index] = time_get() - command_timings_start[cmd_index];
        command_timings_start[cmd_index] = 0;
    }

    switch (cmd_index)
    {
    case 1: {
        const LightCommand& command = std::get<1>(cmd);
        trains_dirty = true;
        for (Train& train : trains) {
            if (train.loco_id == command.loco_id) {
                train.light_on = command.value;
                return;
            }
        }
        break;
    }
    case 2: {
        const SpeedCommand& command = std::get<2>(cmd);
        trains_dirty = true;
        for (Train& train : trains) {
            if (train.loco_id == command.loco_id) {
                train.requested_speed = command.speed;
                return;
            }
        }
        break;
    }
    case 3: {
        const DirectionCommand& command = std::get<3>(cmd);
        trains_dirty = true;
        for (Train& train : trains) {
            if (train.loco_id == command.loco_id) {
                train.backward = command.backward;
                return;
            }
        }
        break;
    }
    case 4: {
        const SwitchCommand& command = std::get<4>(cmd);
        switches_dirty = true;
        if (command.sw_id > 0 && command.sw_id <= 5 * 16) {
            uint16_t shift = command.sw_id - 1 - (command.sw_id > 18 ? 135 : 0);
            if (command.straight) {
                switches &= ~(1 << shift);
            } else {
                switches |= (1 << shift);
            }
        }
    }   break;
    case 5: {
        const SensorData& command = std::get<5>(cmd);
        if (command.new_state) {
            if (sensors.size() == 0 || sensors.peek_last() != command.sensor_id) {
                sensors_dirty = true;
                if (sensors.size() == MAX_SENSORS_RECENT) {
                    sensors.pop();
                }
                sensors.push(command.sensor_id);
            }
        }
        break;
    }
    case 6: {
        const ControlCommand& command = std::get<6>(cmd);
        switch (command.type)
        {
        case ControlCommand::CMD_GO:
            if (stopped) {
                stopped = false;
                status_dirty = true;
            }
            break;
        case ControlCommand::CMD_STOP:
            if (!stopped) {
                stopped = true;
                status_dirty = true;
            }
            break;
        case ControlCommand::CMD_HALT:
            for (Train& train : trains) {
                train.requested_speed = 0;
            }
            break;
        default:
            break;
        }
    }
    default:
        break;
    }
}

void apply_state(const State& state) {
    // stop all trains first
    mcp2515_send(ControlCommand(ControlCommand::CMD_HALT).to_frame());

    for (const Train& train : state.trains) {
        mcp2515_send(LightCommand(train.loco_id, train.light_on).to_frame());
        mcp2515_send(SpeedCommand(train.loco_id, train.requested_speed).to_frame());
        mcp2515_send(DirectionCommand(train.loco_id, train.backward).to_frame());
    }

    for (int sw_id = 0; sw_id < 22; ++sw_id) {
        mcp2515_send(SwitchCommand((sw_id + (sw_id > 17 ? 135 : 0)), state.switches & (1 << sw_id)).to_frame(), sw_id * 100);
    }

    mcp2515_send(ControlCommand(state.stopped ? ControlCommand::CMD_STOP : ControlCommand::CMD_GO).to_frame());
}

void print_state(State& state, bool force) {
    if (state.status_dirty || force) {
        uart_printf(CONSOLE, "\033[%u;2HTrack %s  \n\r", STATUS_ROW, state.stopped ? "Stopped" : "Active");
        state.status_dirty = false;
    }

    // for each train, print the train
    if (state.trains_dirty || force) {
        uart_printf(CONSOLE, "\033[%u;2HTrain | Dir | Lamp | Speed \n\r", TRAIN_ROW);
        for (auto& Train : state.trains) {
            uart_printf(CONSOLE, "\033[K   %u  | %s | %s | %u\n\r", Train.loco_id,
                Train.backward ? "Rev" : "Fwd", Train.light_on ? " On " : " Off", Train.requested_speed);
        }
        state.trains_dirty = false;
    }

    if (state.sensors_dirty || force) {
        uart_printf(CONSOLE, "\033[%u;2HRecent Sensors \n\r\033[K   ", SENSOR_ROW);
        for (size_t i = state.sensors.size(); i-- > 0; ) {
            uint16_t s_id = state.sensors[i];
            char bank = 'A' + (s_id / 16);
            int number = (s_id % 16) + 1;
            uart_printf(CONSOLE, "%c%d ", bank, number);
        }
        uart_puts(CONSOLE, "\n\r");
        state.sensors_dirty = false;
    }

    if (state.switches_dirty || force) {
        uart_printf(CONSOLE, "\033[%u;2HSwitches\n\r", SWITCH_ROW);
        for (int sw_id = 0; sw_id < 22; ++sw_id) {
            const char c = state.switches & (1 << sw_id) ? 'C' : 'S';

            if (sw_id < 9) {
                uart_printf(CONSOLE, "   %u  : %c", sw_id + 1, c);
            } else if (sw_id < 18) {
                uart_printf(CONSOLE, "   %u : %c", sw_id + 1, c);
            } else {
                uart_printf(CONSOLE, "   %u: %c", sw_id + 136, c);
            }
            if (sw_id % 4 == 3) {
                uart_puts(CONSOLE, "\n\r");
            }
        }
        state.switches_dirty = false;
    }

    if (state.timings_dirty || force) {
        uart_printf(CONSOLE, "\033[%u;2HCommand Timings\n\r", TIMING_ROW);
        const char* cmd_names[] = { "Unknown ", "Light   ", "Speed   ", "Dir     ", "Switch  ", "Sensor  ", "Control " };
        for (size_t i = 0; i < MRK_CMD_COUNT; ++i) {
            if (state.command_timings[i] > 0) {
                uart_printf(CONSOLE, "   %s: %u us (%u ms)\n\r", cmd_names[i], state.command_timings[i], state.command_timings[i] / 1000);
            }
        }
        state.timings_dirty = false;
    }
}