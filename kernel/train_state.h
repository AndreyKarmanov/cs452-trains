#pragma once

#include "buffer.h"
#include "mrk.h"
#include <stdint.h>

#define MAX_TRAINS 6
#define MAX_SENSORS_RECENT 10

// std::array<uint32_t, 15> v_max_18{0,   0,   32,  40,  78,  114,  131, 178,
//                                   230, 294, 341, 402, 468, 530, 603};
// std::array<int, 15> accel_18{
//     -47, 47, 47, 47, 47, 47, 47, 47, 47, 192, 50, 110, 38, 44, 47,
// };

struct TrainState {
  uint32_t loco_id;

  uint16_t req_speed{0};

  bool backward : 1 = false;
  bool light_on : 1 = true;

  // units of um/tick (micrometer per tick)
  std::array<int, 15> v_max{0,   0,   32,  40,  79,  111, 131, 178,
                            250, 274, 332, 393, 456, 515, 588};
  // units of nm/ticks^2 (nanometer per tick^2) aka 1000*um / ticks^2
  std::array<int, 15> accel{
      0, 47, 47, 47, 47, 47, 47, 47, 47, 192, 50, 110, 103, 99, 92,
  };

  // units of nm/ticks^2 (nanometer per tick^2) aka 1000*um / ticks^2
  std::array<int, 15> decel{
      0, 47, 47, 47, 47, 47, 47, 47, 47, 192, 50, 110, 103, 99, 92,
  };

  std::array<uint32_t, 15> stop_dist_um{
      1000,   40000,  55000,  80000,  110000, 140000,  18000,   250000,
      340000, 440000, 660000, 770000, 970000, 1140000, 1350000,
  };

  // units of um / tick
  int ve{0};
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
  // Map<int, TrainState, MAX_TRAINS> train_map{};
  std::array<TrainState, MAX_TRAINS> trains{{{13, 0, false, true},
                                             {14, 0, false, true},
                                             {15, 0, false, true},
                                             {17, 0, false, true},
                                             {18, 0, false, true},
                                             {55, 0, false, true}}};

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

  void update_from_mrk(const MRKCmd &cmd, uint32_t tick);
  TrainState *get_loco(uint32_t loco_id) {
    for (auto &train : trains) {
      if (train.loco_id == loco_id) {
        return &train;
      }
    }
    return nullptr;
  }
};
