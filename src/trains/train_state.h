#pragma once

#include "buffer.h"
#include "map.h"
#include "mrk.h"
#include "pathfind.h"
#include <stdint.h>

#define MAX_TRAINS 7
#define MAX_SENSORS_RECENT 10

constexpr int TRAIN_LENGTH_UM = 220'000; // um

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

  // sensors attributed to this train train
  struct SeenSensor {
    SensorData sens;
    uint32_t tick;

    bool operator==(const SeenSensor &other) const = default;
  };
  std::optional<SeenSensor> last_sensor{};

  EncodedPath e_path{};

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
  int d_shoe_um{0};
  int d_shoe_reverse_um{0};

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
  Map<uint8_t, uint32_t, TRACK_MAX> reservations{};

  // Map<int, TrainState, MAX_TRAINS> train_map{};
  std::array<TrainState, MAX_TRAINS> trains{
      {{.id                = 13,
        .req_speed         = 0,
        .backward          = false,
        .light_on          = true,
        .d_shoe_um         = TRAIN_LENGTH_UM / 2,
        .d_shoe_reverse_um = TRAIN_LENGTH_UM / 2},
       {.id         = 10,
        .req_speed  = 0,
        .backward   = false,
        .light_on   = true,
        .v_max_umpt = {0, 10, 30, 53, 66, 98, 135, 185, 233, 288, 346, 413, 480,
                       540, 617},
        .a_nmpt2 = {20, 20, 20, 30, 30, 33, 59, 53, 63, 68, 71, 76, 79, 83, 86},
        .d_nmpt2 = {0, 5, 23, 47, 54, 84, 83, 97, 99, 109, 114, 119, 123, 123,
                    126},
        .stop_params       = {556, 323, 3.43},
        .d_shoe_um         = 50'000,
        .d_shoe_reverse_um = 165'000},
       {.id         = 14,
        .req_speed  = 0,
        .backward   = false,
        .light_on   = true,
        .v_max_umpt = {0, 9, 32, 54, 67, 92, 132, 181, 227, 278, 336, 398, 460,
                       524, 597},
        .a_nmpt2 = {20, 20, 20, 20, 20, 33, 90, 50, 57, 60, 64, 69, 72, 77, 77},
        .d_nmpt2 = {0, 4, 26, 49, 56, 76, 76, 96, 98, 107, 109, 116, 116, 119,
                    119},
        .stop_params       = {705, 272, 3.72},
        .d_shoe_um         = TRAIN_LENGTH_UM / 2,
        .d_shoe_reverse_um = TRAIN_LENGTH_UM / 2},
       {.id         = 15,
        .req_speed  = 0,
        .backward   = false,
        .light_on   = true,
        .v_max_umpt = {0, 8, 32, 50, 78, 94, 130, 176, 222, 273, 328, 389, 450,
                       512, 584},
        .a_nmpt2 = {33, 33, 33, 33, 33, 33, 33, 56, 52, 57, 63, 64, 71, 75, 78},
        .d_nmpt2 = {33, 1, 11, 20, 33, 40, 53, 66, 76, 85, 92, 99, 104, 108,
                    112},
        .stop_params       = {26000, 582, 3.4},
        .d_shoe_um         = TRAIN_LENGTH_UM / 2,
        .d_shoe_reverse_um = TRAIN_LENGTH_UM / 2},
       {.id         = 17,
        .req_speed  = 0,
        .backward   = false,
        .light_on   = true,
        .v_max_umpt = {0, 9, 32, 54, 67, 97, 136, 185, 233, 283, 342, 407, 472,
                       529, 610},
        .a_nmpt2 = {20, 20, 20, 20, 20, 33, 35, 43, 54, 63, 68, 70, 76, 80, 82},
        .d_nmpt2 = {0, 4, 17, 49, 56, 102, 110, 112, 111, 107, 111, 117, 121,
                    121, 124},
        .stop_params       = {910, 235, 3.66},
        .d_shoe_um         = 84'000,
        .d_shoe_reverse_um = 162'000},
       {.id         = 18,
        .req_speed  = 0,
        .backward   = false,
        .light_on   = true,
        .v_max_umpt = {0, 9, 32, 54, 67, 101, 139, 186, 240, 295, 353, 419, 488,
                       556, 624},
        .a_nmpt2 = {20, 20, 20, 20, 20, 29, 63, 63, 62, 68, 75, 77, 80, 84, 85},
        .d_nmpt2 = {0, 4, 26, 49, 75, 89, 83, 96, 105, 109, 115, 118, 124, 126,
                    126},
        .stop_params       = {716, 314, 3.44},
        .d_shoe_um         = 57'000,
        .d_shoe_reverse_um = 164'000},
       {
           .id                = 55,
           .req_speed         = 0,
           .backward          = false,
           .light_on          = true,
           .stop_params       = {2090, 908, 3},
           .d_shoe_um         = TRAIN_LENGTH_UM / 2,
           .d_shoe_reverse_um = TRAIN_LENGTH_UM / 2,
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

// Returns location on the train's current path at d_um from the first node.
std::optional<PathLocation> locate_train(const Track &track,
                                         const TrainState &train);

int train_head_um(const TrainState &train);
int train_tail_um(const TrainState &train);
