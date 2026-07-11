#pragma once

#include "buffer.h"
#include "mrk.h"
#include "static_string.h"
#include <stdint.h>

#define MAX_TRAINS 6
#define MAX_SENSORS_RECENT 10

// Train 15 Stats:
// Speed,Top,Accel,Decel,Stop Dist
//   0   ,   0 ,     0 ,     0 , 1
//   1   ,   8 ,     0 ,     0 , 40
//   2   ,  32 ,     0 ,     0 , 55
//   3   ,  50 ,    33 ,    33 , 80
//   4   ,  78 ,    33 ,    33 , 90
//   5   ,  94 ,    33 ,    33 , 100
//   6   , 130 ,    33 ,    33 , 154
//   7   , 176 ,    56 ,    56 , 249
//   8   , 222 ,    52 ,    52 , 336
//   9   , 273 ,    57 ,    57 , 391
//  10   , 328 ,    63 ,    63 , 501
//  11   , 389 ,    64 ,    64 , 683
//  12   , 450 ,    71 ,    71 , 833
//  13   , 512 ,    75 ,    75 , 1055
//  14   , 584 ,    78 ,    78 , 1307

struct TrainState {
  uint32_t loco_id;

  uint16_t req_speed{0};

  bool backward : 1 = false;
  bool light_on : 1 = true;

  // Todo: use this to let train controller where train starts.
  StaticString<8> init_sensor{};

  // units of um/tick (micrometer per tick)
  std::array<int, 15> v_max_umpt{
      0, 8, 32, 50, 78, 94, 130, 176, 222, 273, 328, 389, 450, 512, 584,
  };

  // units of nm/ticks^2 (nanometer per tick^2) aka 1000*um / ticks^2
  std::array<int, 15> a_nmpt2{33, 33, 33, 33, 33, 33, 33, 56,
                              52, 57, 63, 64, 71, 75, 78};

  // units of -nm/ticks^2 (nanometer per tick^2) aka 1000*um / ticks^2
  // these are wrong values right now, I forgot to ca
  std::array<int, 15> d_nmpt2{33, 33, 33, 33, 33, 33, 33, 56,
                              52, 57, 63, 64, 71, 75, 78};

  std::array<uint32_t, 15> stop_dist_um{
      1000,   40000,  55000,  80000,  90000,  100000,  154000,  249000,
      336000, 391000, 501000, 683000, 833000, 1055000, 1307000,
  };

  // std::array<uint32_t, 15> stop_dist_um{
  //     1000,   40000,  55000,  80000,  110000, 140000,  18000,   250000,
  //     340000, 440000, 660000, 770000, 970000, 1140000, 1350000,
  // };

  // units of nm / tick
  int ve_nm{0};
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
