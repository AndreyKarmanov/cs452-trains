#ifndef _state_h_
#define _state_h_ 1

#include <stdint.h>
#include "can.h"
#include "buffer.h"

#define MAX_TRAINS 6
#define MAX_SENSORS_RECENT 10

struct Train
{
    uint32_t loco_id;

    uint16_t requested_speed;

    bool backward : 1;
    bool light_on : 1;
};

struct State
{
    static constexpr uint32_t SWITCH_COUNT = 22;
    static constexpr uint32_t SWITCH_MASK = (1u << SWITCH_COUNT) - 1u;

    static constexpr uint16_t switch_index(uint16_t sw_id) {
        return sw_id > 18 ? sw_id - 135 : sw_id - 1;
    }

    static constexpr uint16_t switch_id(uint16_t index) {
        return index > 17 ? index + 135 : index + 1;
    }

    static constexpr bool is_switch_id(uint16_t sw_id) {
        return (sw_id >= 1 && sw_id <= 18) || (sw_id >= 153 && sw_id <= 156);
    }

    static constexpr uint32_t switch_bit(uint16_t sw_id) {
        return 1u << switch_index(sw_id);
    }

    bool is_switch_straight(uint16_t sw_id) const {
        return (switches & switch_bit(sw_id)) != 0;
    }

    void set_switch(uint16_t sw_id, bool straight) {
        if (straight) {
            switches |= switch_bit(sw_id);
        } else {
            switches &= ~switch_bit(sw_id);
        }
    }

    // switches, both tracks have same amount
    // switches[0:17] = 1..18
    // switches[18:21] = 153..156
    // bit set means straight
    uint32_t switches = SWITCH_MASK;

    // recent sensors
    Buffer<uint16_t, MAX_SENSORS_RECENT> sensors{};

    // trains
    Train trains[MAX_TRAINS]{
        { 13, 0, false, true },
        { 14, 0, false, true },
        { 15, 0, false, true },
        { 17, 0, false, true },
        { 18, 0, false, true },
        { 55, 0, false, true }
    };

    // track go / stop
    bool stopped : 1 = true;
    bool trains_dirty : 1 = true;
    bool switches_dirty : 1 = true;
    bool sensors_dirty : 1 = true;
    bool status_dirty : 1 = true;
    bool timings_dirty : 1 = true;

    uint32_t command_timings_start[MRK_CMD_COUNT] = { 0 };
    uint32_t command_timings[MRK_CMD_COUNT] = { };

    void update_from_mrk(const MRK_CMD& cmd);
    Train get_loco(uint32_t loco_id) const {
        for (const Train& train : trains) {
            if (train.loco_id == loco_id) {
                return train;
            }
        }
        return Train{ loco_id, 0, false, false };
    }
};

void apply_state(const State& state);
void print_state(State& state, bool force = 0);

#endif // _state_h_