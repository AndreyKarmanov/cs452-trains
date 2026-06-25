#pragma once

#include "buffer.h"
#include "mrk.h"
#include <stdint.h>

#define MAX_TRAINS 6
#define MAX_SENSORS_RECENT 10

struct TrainState {
  uint32_t loco_id;

  uint16_t requested_speed;

  bool backward : 1;
  bool light_on : 1;

  // uinits of 0.001 mm/tick (micrometer per tick)
  std::array<uint16_t, 15> top_speed{0,   0,   32,  40,  80,  100, 140, 200,
                                     250, 310, 360, 440, 500, 532, 605};
  std::array<uint16_t, 15> loop_time{0,   0,   32,  40,  80,  100, 140,  200,
                                     250, 310, 360, 440, 500, 532, 15401};
  // uinits of um per 1kticks^2)
  uint16_t accel = 47;
};

struct State {

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
  // REMEMBER IT"S IN REVERSE!!!!
  // uint32_t switches = 0b11110111110101111010100000000000;
  uint32_t switches = 0b00000000000'00000'1000'0010'0000'0000;

  // recent sensors
  Buffer<uint16_t, MAX_SENSORS_RECENT> sensors{};

  // trains
  TrainState trains[MAX_TRAINS]{{13, 0, false, true}, {14, 0, false, true},
                                {15, 0, false, true}, {17, 0, false, true},
                                {18, 0, false, true}, {55, 0, false, true}};

  // track go / stop
  bool stopped : 1        = true;
  bool trains_dirty : 1   = true;
  bool switches_dirty : 1 = true;
  bool sensors_dirty : 1  = true;
  bool status_dirty : 1   = true;

  bool is_dirty() const {
    return trains_dirty || switches_dirty || sensors_dirty || status_dirty;
  };

  void clear_dirty() {
    trains_dirty   = false;
    switches_dirty = false;
    sensors_dirty  = false;
    status_dirty   = false;
  }

  void update_from_mrk(const MRKCmd &cmd);
  TrainState get_loco(uint32_t loco_id) const {
    for (const TrainState &train : trains) {
      if (train.loco_id == loco_id) {
        return train;
      }
    }
    return TrainState{loco_id, 0, false, false};
  }
};
