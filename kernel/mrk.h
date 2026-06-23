#pragma once

#include <cstddef>
#include <stdint.h>
#include <variant>

// 1100 0011 0000 0000
constexpr uint16_t MRK_HASH = 0xC300;

// this is an intermediate format that matches the MRK diagram / frame on the
// bus, for easy debugging
struct CANFRAME {

  // bitfields don't do much here, I mostly included these as a reminder for
  // what size these are.
  uint32_t prio : 4 = 0;
  uint32_t cmdid : 8;
  uint32_t resp : 1  = 0;
  uint32_t hash : 16 = MRK_HASH;

  uint8_t dlc : 4;

  uint8_t data[8];

  void encode_data_0_4(uint32_t value) {
    data[0] = (value >> 24) & 0xFF;
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;
  }

  uint32_t decode_data_0_4() const {
    return (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) |
           (uint32_t(data[2]) << 8) | uint32_t(data[3]);
  }
};

struct LightCmd {
  static constexpr uint8_t cmdid = 0x06;

  uint32_t loco_id;
  bool value;

  LightCmd(uint32_t loco_id, bool value) : loco_id(loco_id), value(value) {}

  LightCmd(const CANFRAME &frame)
      : loco_id(frame.decode_data_0_4()), value(frame.data[5] != 0) {}

  CANFRAME to_frame() const {
    CANFRAME frame;

    frame.cmdid = cmdid;
    frame.dlc   = 6;
    frame.encode_data_0_4(loco_id);
    frame.data[4] = 0;
    frame.data[5] = value;

    return frame;
  }
};

struct SpeedCmd {
  static constexpr uint8_t cmdid = 0x04;

  uint32_t loco_id;
  uint16_t speed;

  SpeedCmd(uint32_t loco_id, uint16_t speed) : loco_id(loco_id), speed(speed) {}

  SpeedCmd(const CANFRAME &frame)
      : loco_id(frame.decode_data_0_4()),
        speed((frame.data[4] << 8) | frame.data[5]) {}

  CANFRAME to_frame() const {
    CANFRAME frame;

    frame.cmdid = cmdid;
    frame.dlc   = 6;
    frame.encode_data_0_4(loco_id);
    frame.data[4] = (speed >> 8) & 0xFF;
    frame.data[5] = speed & 0xFF;

    return frame;
  }
};

struct DirectionCmd {
  static constexpr uint8_t cmdid = 0x05;

  uint32_t loco_id;
  bool backward;

  DirectionCmd(uint32_t loco_id, bool backward)
      : loco_id(loco_id), backward(backward) {}

  DirectionCmd(const CANFRAME &frame)
      : loco_id(frame.decode_data_0_4()), backward(frame.data[4] != 1) {}

  CANFRAME to_frame() const {
    CANFRAME frame;

    frame.cmdid = cmdid;
    frame.dlc   = 5;
    frame.encode_data_0_4(loco_id);
    frame.data[4] = backward + 1;

    return frame;
  }
};

struct SwitchCmd {
  static constexpr uint8_t cmdid = 0x0B;

  uint16_t sw_id;
  bool straight;

  SwitchCmd(uint16_t sw_id, bool straight) : sw_id(sw_id), straight(straight) {}

  SwitchCmd(const CANFRAME &frame)
      : sw_id(uint16_t(frame.decode_data_0_4() - 0x3000 + 1)),
        straight(frame.data[4] != 0) {}

  CANFRAME to_frame() const {
    CANFRAME frame;

    frame.cmdid = cmdid;
    frame.dlc   = 6;

    // sw_id is 1-indexed in diagram, but can is 0-indexed.
    frame.encode_data_0_4(0x3000 + sw_id - 1);
    frame.data[4] = straight;
    frame.data[5] = 1;

    return frame;
  }
};

struct SensorData {
  static constexpr uint8_t cmdid = 0x11;

  uint16_t sensor_id;

  uint8_t bank;
  uint8_t number;

  bool old_state;
  bool new_state;

  SensorData(const CANFRAME &frame)
      : sensor_id((frame.decode_data_0_4() & 0xFFFF)), old_state(frame.data[4]),
        new_state(frame.data[5]) {
    bank   = ((sensor_id - 1) / 16);
    number = ((sensor_id - 1) % 16) + 1;
  }
  CANFRAME to_frame() const { return {}; }
};

struct ControlCmd {
  static constexpr uint8_t cmdid = 0x00;

  typedef enum {
    CMD_STOP          = 0x00,
    CMD_GO            = 0x01,
    CMD_HALT          = 0x02,
    CMD_REMOVE_TRAINS = 0x04
  } CmdType;

  CmdType type;

  ControlCmd(CmdType type) : type(type) {}
  ControlCmd(const CANFRAME &frame)
      : type(static_cast<CmdType>(frame.data[4])) {}

  CANFRAME to_frame() const {
    CANFRAME frame;

    frame.cmdid = cmdid;
    frame.dlc   = 5;

    for (int i = 0; i < 3; ++i) {
      frame.data[i] = 0;
    }

    frame.data[4] = type;

    return frame;
  }
};

struct UnknownCmd {
  CANFRAME frame;
  UnknownCmd() = default;
  UnknownCmd(const CANFRAME &frame) : frame(frame) {}
  CANFRAME to_frame() const { return {}; }
};

namespace UserCmd {
  struct Invalid {};
  struct Quit {};
  struct Light {
    uint32_t id;
    bool flag;
  };
  struct Speed {
    uint32_t id;
    uint32_t value;
  };
  struct Switch {
    uint32_t id;
    bool flag;
  };
  struct Reverse {
    uint32_t id;
    bool flag;
  };
  struct Stop {};
  struct Go {};
  struct Reset {};
  struct RemoveTrains {};
  struct RunTree {
    uint32_t id;
    uint32_t value;
  };

  struct CalSpeed {
    uint32_t id;
    uint32_t value;
  };

  using Cmd = std::variant<Invalid, Quit, Light, Speed, Switch, Reverse, Stop,
                           Go, Reset, RemoveTrains, RunTree, CalSpeed>;

  constexpr size_t COUNT = std::variant_size<Cmd>::value;
} // namespace UserCmd

using MRKCmd = std::variant<UnknownCmd, LightCmd, SpeedCmd, DirectionCmd,
                            SwitchCmd, SensorData, ControlCmd>;

constexpr size_t MRK_CMD_COUNT = std::variant_size<MRKCmd>::value;

inline CANFRAME encode_frame(const MRKCmd &cmd) {
  CANFRAME frame{};
  std::visit([&](auto &&arg) { frame = arg.to_frame(); }, cmd);
  return frame;
};

inline MRKCmd decode_frame(const CANFRAME &frame) {
  switch (frame.cmdid) {
  case LightCmd::cmdid:
    return LightCmd(frame);
  case SpeedCmd::cmdid:
    return SpeedCmd(frame);
  case DirectionCmd::cmdid:
    return DirectionCmd(frame);
  case SwitchCmd::cmdid:
    return SwitchCmd(frame);
  case SensorData::cmdid:
    return SensorData(frame);
  case ControlCmd::cmdid:
    return ControlCmd(frame);
  default:
    return UnknownCmd(frame);
  }
}

inline bool is_mrk_response_to(const CANFRAME &sent, const CANFRAME &recv) {
  if (sent.resp != 0 || recv.resp != 1) {
    return false;
  }
  if (sent.cmdid != recv.cmdid || sent.dlc != recv.dlc) {
    return false;
  }
  for (uint8_t i = 0; i < sent.dlc; ++i) {
    if (sent.data[i] != recv.data[i]) {
      return false;
    }
  }
  return true;
}
