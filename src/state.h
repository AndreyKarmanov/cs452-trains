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

    // switches, both tracks have same amount
    // switches[0:17] = 1..18
    // switches[18:21] = 153..156
    uint32_t switches = 0;

    // recent sensors
    Buffer<uint16_t, MAX_SENSORS_RECENT> sensors{};

    // trains
    Train trains[MAX_TRAINS]{
        { 13, 0, false, true },
        { 14, 500, false, false },
        { 15, 500, true, false },
        { 17, 0, false, false },
        { 18, 0, false, false },
        { 55, 0, false, false }
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