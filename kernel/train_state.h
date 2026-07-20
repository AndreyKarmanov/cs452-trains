#pragma once

#include "buffer.h"
#include "mrk.h"
#include "pathfind.h"
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

// spd,vmax,a,d
// 2,33,0,33
// 3,44,0,33
// 4,66,33,0
// 5,98,598,0
// 6,137,56,4
// 7,185,62,10
// 8,232,67,18
// 9,286,66,30
// 10,343,74,44
// 11,405,78,58
// 12,470,81,72
// 13,534,84,85
// 14,607,86,94

struct TrainState {
  uint32_t id;

  uint16_t req_speed{0};

  bool backward : 1 = false;
  bool light_on : 1 = true;

  int inital_node_idx{-1};

  // sensors attributed to this train train
  struct SeenSensor {
    SensorData sens;
    uint32_t tick;
  };
  std::optional<SeenSensor> last_sensor{};
  bool reversed_since_last_sensor{false};

  EncodedPath e_path{};

  // units of um/tick (micrometer per tick)
  std::array<int, 15> v_max_umpt{
      0, 8, 32, 50, 78, 94, 130, 176, 222, 273, 328, 389, 450, 512, 584,
  };

  // units of nm/ticks^2 (nanometer per tick^2) aka 1000*um / ticks^2
  std::array<int, 15> a_nmpt2{
      33, 33, 33, 33, 33, 33, 33, 56, 52, 57, 63, 64, 71, 75, 78,
  };

  // units of -nm/ticks^2 (nanometer per tick^2) aka 1000*um / ticks^2
  std::array<int, 15> d_nmpt2{
      33, 1, 11, 20, 33, 40, 53, 66, 76, 85, 92, 99, 104, 108, 112,
  };

  // manually determined
  // std::array<uint32_t, 15> stop_dist_um{
  //     0,      40000,  55000,  80000,  90000,  100000,  154000,  249000,
  //     336000, 391000, 501000, 683000, 833000, 1055000, 1307000,
  // };

  // units of nm / tick
  uint64_t ve_nm{0};

  // delta x
  int d_um{0};

  // stop dist
  int stop_dist_um{0};
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

  bool is_switch_curved(uint16_t sw_id) const {
    return (switches & switch_bit(sw_id)) == 0;
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
      if (train.id == loco_id) {
        return &train;
      }
    }
    return nullptr;
  }
};
