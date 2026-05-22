#ifndef _state_h_
#define _state_h_ 1

#include <stdint.h>
#include "can.h"

struct Train
{
    uint32_t loco_id;

    uint16_t requested_speed;
    bool backward;
    bool light_on;
};

struct State
{
    // track go / stop
    bool stopped;

    // trains
    Train trains[6]{
        { 13, 0, false, true },
        { 14, 500, false, false },
        { 15, 500, true, false },
        { 17, 0, false, false },
        { 18, 0, false, false },
        { 55, 0, false, false }
    };

    // switches, both tracks have same amount
    // switches[0:17] = 1..18
    // switches[18:21] = 153..156
    bool switches[22]{ 0 };

    // recent sensors
    uint32_t sensors[10]{ 0 };

    void update_from_mrk(const MRK_CMD& cmd);
};

void apply_state(const State& state);

#endif // _state_h_