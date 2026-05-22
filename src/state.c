#include "state.h"
#include "can.h"
#include "mcp2515.h"

void State::update_from_mrk(const MRK_CMD& cmd) {
    switch (cmd.index())
    {
    case 1: {
        const LightCommand& command = std::get<1>(cmd);
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
        if (command.sw_id > 0 && command.sw_id <= 5 * 16) {
            switches[command.sw_id - 1] = command.straight;
        }
    }   break;
    case 5: {
        const SensorData& command = std::get<5>(cmd);
        // need to display the last few sensors
        // should implement a circular queue TBH. 
        break;
    }
    case 6: {
        const ControlCommand& command = std::get<6>(cmd);
        switch (command.type)
        {
        case ControlCommand::CMD_GO:
            stopped = false;
            break;
        case ControlCommand::CMD_STOP:
            stopped = true;
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
        mcp2515_send(SwitchCommand((sw_id + sw_id > 17 ? 135 : 0), state.switches[sw_id]).to_frame());
    }

    mcp2515_send(ControlCommand(state.stopped ? ControlCommand::CMD_STOP : ControlCommand::CMD_GO).to_frame());
}