#ifndef _can_h_
#define _can_h_ 1

#include <stdint.h>

// 1100 0011 0000 0000
constexpr uint16_t MRK_HASH = 0xC300;


// frame is structed in same way the buffer is laid out
// this relies on the memory being laid out in the same way
// this can potentially cause issues if flags change, compiler changes, hardware, etc.
struct TXBnFrame {

    union
    {
        // we write the TXBnCTRL register separately, so we need a byte-level view
        // otherwise we just write the full TXBnFrame starting from SIDH to DATA8
        uint8_t byte;
        struct {
            uint8_t TXP0 : 1;
            uint8_t TXP1 : 1;
            uint8_t : 1;
            uint8_t TXREQ : 1;
            uint8_t TXERR : 1;
            uint8_t MLOA : 1;
            uint8_t ABTF : 1;
            uint8_t : 1;
        } bits;
    } CTRL;

    uint8_t SIDH;   // SID[10:3]

    union
    {
        uint8_t byte;
        struct {
            uint8_t EID_17_16 : 2;
            uint8_t : 1;
            uint8_t EXIDE : 1;
            uint8_t : 1;
            uint8_t SID_2_0 : 3;
        } bits;
    } SIDL;

    uint8_t EID8;   // EID[15:8]
    uint8_t EID0;   // EID[7:0]

    union
    {
        uint8_t byte;
        struct {
            uint8_t DLC : 4;
            uint8_t : 2;
            uint8_t RTR : 1;
            uint8_t : 1;
        } bits;
    } DLC;

    uint8_t    data[8];
};


struct RXBnFRAME {

    // the ctrl are diff for the two buffers
    // we just need to make sure RXM is set to 0b11 and BUKT is 1
    // this is to remove filters and ensure rollover is enabled
    union
    {
        uint8_t byte;
        struct {
            uint8_t : 2;
            uint8_t BUKT : 1; // should set/read 1 every time
            uint8_t RXRTR : 1;
            uint8_t : 1;
            uint8_t RXM : 2; // should set/read 0b11 always
            uint8_t : 1;
        } bits;
    } CTRL;

    uint8_t SIDH;   // SID[10:3]
    
    union
    {
        uint8_t byte;
        struct {
            uint8_t EID_17_16 : 2;
            uint8_t : 1;
            uint8_t IDE : 1; // EXIDE equiv, should be 1 always
            uint8_t SRR : 1;
            uint8_t SID_2_0 : 3;
        } bits;
    } SIDL;

    uint8_t EID8;   // EID[15:8]
    uint8_t EID0;   // EID[7:0]

    union
    {
        uint8_t byte;
        struct {
            uint8_t DLC : 4;
            uint8_t : 2;
            uint8_t RTR : 1;
            uint8_t : 1;
        } bits;
    } DLC;

    uint8_t data[8];
};

struct CANFRAME {

    // bitfields don't do much here, I mostly included these as a reminder for what size these are. 
    uint32_t prio : 4 = 0;
    uint32_t cmdid : 8;
    uint32_t resp : 1 = 0;
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
        return (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | uint32_t(data[3]);
    }
};

struct LightCommand {
    static constexpr uint8_t cmdid = 0x06;

    uint32_t loco_id;
    bool value;

    LightCommand(uint32_t loco_id, bool value) : loco_id(loco_id), value(value) {}

    LightCommand(const CANFRAME& frame) : loco_id(frame.decode_data_0_4()), value(frame.data[5] != 0) {}

    CANFRAME to_frame() const {
        CANFRAME frame;

        frame.cmdid = cmdid;
        frame.dlc = 6;
        frame.encode_data_0_4(loco_id);
        frame.data[4] = 0;
        frame.data[5] = value;

        return frame;
    }
};

struct SpeedCommand {
    static constexpr uint8_t cmdid = 0x04;

    uint32_t loco_id;
    uint16_t speed;

    SpeedCommand(uint32_t loco_id, uint16_t speed) : loco_id(loco_id), speed(speed) {}

    SpeedCommand(const CANFRAME& frame)
        : loco_id(frame.decode_data_0_4()), speed((frame.data[4] << 8) | frame.data[5]) {}

    CANFRAME to_frame() const {
        CANFRAME frame;

        frame.cmdid = cmdid;
        frame.dlc = 6;
        frame.encode_data_0_4(loco_id);
        frame.data[4] = (speed >> 8) & 0xFF;
        frame.data[5] = speed & 0xFF;

        return frame;
    }
};

struct DirectionCommand {
    static constexpr uint8_t cmdid = 0x05;

    uint32_t loco_id;
    bool backward;

    DirectionCommand(uint32_t loco_id, bool backward) : loco_id(loco_id), backward(backward) {}

    DirectionCommand(const CANFRAME& frame) : loco_id(frame.decode_data_0_4()), backward(frame.data[4] != 1) {}

    CANFRAME to_frame() const {
        CANFRAME frame;

        frame.cmdid = cmdid;
        frame.dlc = 5;
        frame.encode_data_0_4(loco_id);
        frame.data[4] = backward + 1;

        return frame;
    }
};

struct SwitchCommand {
    static constexpr uint8_t cmdid = 0x0B;

    uint16_t sw_id;
    bool straight;

    SwitchCommand(uint16_t sw_id, bool straight) : sw_id(sw_id), straight(straight) {}

    SwitchCommand(const CANFRAME& frame)
        : sw_id(uint16_t(frame.decode_data_0_4() - 0x3000 + 1)), straight(frame.data[4] != 0) {}

    CANFRAME to_frame() const {
        CANFRAME frame;

        frame.cmdid = cmdid;
        frame.dlc = 6;

        // sw_id is 1-indexed in diagram, but can is 0-indexed.
        frame.encode_data_0_4(0x3000 + sw_id - 1);
        frame.data[4] = straight;
        frame.data[5] = 1;

        return frame;
    }
};

struct SensorData {
    static constexpr uint8_t cmdid = 0x0A;

    uint32_t sensor_id;
    bool value;

    SensorData(uint32_t sensor_id, bool value) : sensor_id(sensor_id), value(value) {}

    SensorData(const CANFRAME& frame) : sensor_id(frame.decode_data_0_4()), value(frame.data[4] != 0) {}

    CANFRAME to_frame() const {
        CANFRAME frame;

        frame.cmdid = cmdid;
        frame.dlc = 5;
        frame.encode_data_0_4(sensor_id);
        frame.data[4] = value;

        return frame;
    }
};


#endif /* _can_h_ */

