#pragma once

#include "buffer.h"
#include "map.h"
#include "mrk.h"
#include "pathfind.h"
#include <cstdint>
#include <stdint.h>

#define MAX_TRAINS 6
#define MAX_SENSORS_RECENT 10

// Train 14 Stats:
// Speed,Top,Accel,Decel
//   0   ,   0 ,    33 ,    33
//   1   ,   8 ,    33 ,    33
//   2   ,  32 ,    33 ,    33
//   3   ,  50 ,    33 ,    33
//   4   ,  78 ,    33 ,    33
//   5   ,  94 ,    33 ,    33
//   6   , 130 ,    33 ,    43
//   7   , 176 ,    56 ,    59
//   8   , 222 ,    52 ,    68
//   9   , 273 ,    57 ,    77
//  10   , 328 ,    63 ,    82
//  11   , 389 ,    64 ,    89
//  12   , 450 ,    71 ,    92
//  13   , 512 ,    75 ,    97
//  14   , 586 ,    80 ,   112
//
// Train 15 Stats (defaults):
// Speed,Top,Accel,Decel
//   0   ,   0 ,    33 ,    33
//   1   ,   8 ,    33 ,     1
//   2   ,  32 ,    33 ,    11
//   3   ,  50 ,    33 ,    20
//   4   ,  78 ,    33 ,    33
//   5   ,  94 ,    33 ,    40
//   6   , 130 ,    33 ,    53
//   7   , 176 ,    56 ,    66
//   8   , 222 ,    52 ,    76
//   9   , 273 ,    57 ,    85
//  10   , 328 ,    63 ,    92
//  11   , 389 ,    64 ,    99
//  12   , 450 ,    71 ,   104
//  13   , 512 ,    75 ,   108
//  14   , 584 ,    78 ,   112
//
// Train 17 Stats:
// Speed,Top,Accel,Decel
//   0   ,   0 ,    33 ,    27
//   1   ,   8 ,    33 ,    27
//   2   ,  32 ,    33 ,    27
//   3   ,  50 ,    33 ,    28
//   4   ,  65 ,    33 ,    44
//   5   ,  97 ,    33 ,    34
//   6   , 134 ,    60 ,    47
//   7   , 181 ,    62 ,    61
//   8   , 230 ,    63 ,    73
//   9   , 283 ,    67 ,    83
//  10   , 339 ,    73 ,    88
//  11   , 402 ,    75 ,    96
//  12   , 465 ,    80 ,    98
//  13   , 528 ,    82 ,   101
//  14   , 598 ,    84 ,   114
//
// Stop dist (um): c0 + c1*v + c2*v^2
// Train 14: -1200 + 1120*v + 2.8*v^2  (-1.2 + 1.12*v + 2.75*v^2 mm)
// Train 15: 26000 + 582*v + 3.4*v^2
// Train 17: 5190 + 908*v + 3*v^2  (5.19 + 0.908*v + 3*v^2 mm)

struct Reservation {
  uint8_t node_idx : 7 {0};
  bool edge_dir : 1 {0};

  bool operator==(const Reservation &other) const {
    return node_idx == other.node_idx && edge_dir == other.edge_dir;
  }
};
struct ReservationHasher {
  constexpr size_t operator()(const Reservation &r) const noexcept {
    return (static_cast<size_t>(r.node_idx) << 1) |
           static_cast<size_t>(r.edge_dir);
  }
};

struct StopParams {
  int c0;
  int c1;
  double c2;
  bool operator==(const StopParams &other) const = default;
};

struct TrainState {
  uint32_t id;

  uint16_t req_speed{0};

  bool backward : 1 = false;
  bool light_on : 1 = true;

  int inital_node_idx{-1};
  int extra_delay{0};

  // sensors attributed to this train train
  struct SeenSensor {
    SensorData sens;
    uint32_t tick;

    bool operator==(const SeenSensor &other) const = default;
  };
  std::optional<SeenSensor> last_sensor{};

  EncodedPath e_path{};
  int res_dist_um{0};

  // units of um/tick (micrometer per tick) with train 15 defaults
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

  // stop dist (um): c0 + c1*ve_um + c2*ve_um^2
  StopParams stop_params{26000, 582, 3.4};

  // units of nm / tick
  uint64_t ve_nm{0};

  // delta x
  int d_um{0};

  // stop dist
  int stop_dist_um{0};

  bool operator==(const TrainState &other) const = default;
};

struct TrackState {

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
  Map<Reservation, uint32_t, TRACK_MAX, ReservationHasher> reservations{};

  // Map<int, TrainState, MAX_TRAINS> train_map{};
  std::array<TrainState, MAX_TRAINS> trains{
      {{.id = 13, .req_speed = 0, .backward = false, .light_on = true},
       {.id          = 14,
        .req_speed   = 0,
        .backward    = false,
        .light_on    = true,
        .extra_delay = 0,
        .v_max_umpt  = {0, 8, 32, 50, 78, 94, 130, 176, 222, 273, 328, 389, 450,
                        512, 586},
        .a_nmpt2 = {33, 33, 33, 33, 33, 33, 33, 56, 52, 57, 63, 64, 71, 75, 80},
        .d_nmpt2 = {33, 33, 33, 33, 33, 33, 43, 59, 68, 77, 82, 89, 92, 97,
                    112},
        .stop_params = {1500, 1100, 2.8}},
       {.id          = 15,
        .req_speed   = 0,
        .backward    = false,
        .light_on    = true,
        .extra_delay = 3,

        .v_max_umpt = {0, 8, 32, 50, 78, 94, 130, 176, 222, 273, 328, 389, 450,
                       512, 584},
        .a_nmpt2 = {33, 33, 33, 33, 33, 33, 33, 56, 52, 57, 63, 64, 71, 75, 78},
        .d_nmpt2 = {33, 1, 11, 20, 33, 40, 53, 66, 76, 85, 92, 99, 104, 108,
                    112},
        .stop_params = {26000, 582, 3.4}},
       {.id          = 17,
        .req_speed   = 0,
        .backward    = false,
        .light_on    = true,
        .extra_delay = 6,
        .v_max_umpt  = {0, 8, 32, 50, 65, 97, 134, 181, 230, 283, 339, 402, 465,
                        528, 598},
        .a_nmpt2 = {33, 33, 33, 33, 33, 33, 60, 62, 63, 67, 73, 75, 80, 82, 84},
        .d_nmpt2 = {27, 27, 27, 28, 44, 34, 47, 61, 73, 83, 88, 96, 98, 101,
                    114},
        .stop_params = {5190, 908, 3}},
       {.id          = 18,
        .req_speed   = 0,
        .backward    = false,
        .light_on    = true,
        .extra_delay = 9},
       {
           .id          = 55,
           .req_speed   = 0,
           .backward    = false,
           .light_on    = true,
           .extra_delay = 12,
       }}};

  // track go / stop
  bool stopped : 1 = true;

  void update(const MRKCmd &cmd);

  TrainState *get_loco(uint32_t loco_id) {
    for (auto &train : trains) {
      if (train.id == loco_id) {
        return &train;
      }
    }
    return nullptr;
  }
};

// Returns location on the train's effective path (last_sensor-prefixed) at
// d_um.
std::optional<PathLocation> locate_train(const Track &track,
                                         const TrainState &train);
