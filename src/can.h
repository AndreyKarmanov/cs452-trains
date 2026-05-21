#ifndef _can_h_
#define _can_h_ 1

#include <stdint.h>

// 1100 0011 0000 0000
constexpr uint16_t MRK_HASH = 0xC300;

struct TXBnCTRL {
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
    };
};

struct TXBnSID {
    uint8_t H;   // SID[10:3]
    uint8_t L;  // SID[2:0] in bits 7:5, EXIDE in bit 3, EID[17:16] in bits 1:0
};

struct TXBnEID {
    uint8_t EID8;   // EID[15:8]
    uint8_t EID0;   // EID[7:0]
};

struct TXBnDLC {
    uint8_t DLC : 4;
    uint8_t : 2;
    uint8_t RTR : 1;
    uint8_t : 1;
};


struct TXBnFrame {
    TXBnCTRL   ctrl{ ctrl.byte = 0 };
    TXBnSID    sid;
    TXBnEID    eid;
    TXBnDLC    dlc;
    uint8_t    data[8];
};


struct CANFRAME {
    uint32_t prio : 4 = 0;
    uint32_t cmdid : 8;
    uint32_t resp : 1 = 0;
    uint32_t hash : 16 = MRK_HASH;

    uint16_t dlc : 4;

    uint8_t data[8];


    void encode_data_0_4(uint32_t value) {
        data[0] = (value >> 24) & 0xFF;
        data[1] = (value >> 16) & 0xFF;
        data[2] = (value >> 8) & 0xFF;
        data[3] = value & 0xFF;
    }
};

struct LightCommand
{
    CANFRAME frame;

    LightCommand(uint32_t loco_id, bool value) {

        frame.cmdid = 0x06;

        frame.dlc = 6;

        frame.encode_data_0_4(loco_id);

        frame.data[4] = 0;
        frame.data[5] = value;
    };
};

struct SpeedCommand
{
    CANFRAME frame;

    SpeedCommand(uint32_t loco_id, uint16_t speed) {
        frame.cmdid = 0x04;

        frame.dlc = 6;

        frame.encode_data_0_4(loco_id);

        frame.data[4] = (speed >> 8) & 0xFF;
        frame.data[5] = speed & 0xFF;
    };
};

struct DirectionCommand
{
    CANFRAME frame;

    DirectionCommand(uint32_t loco_id, bool backward) {
        frame.cmdid = 0x05;

        frame.dlc = 5;

        frame.encode_data_0_4(loco_id);

        frame.data[4] = backward + 1;
    };
};

struct SwitchCommand
{
    CANFRAME frame;

    SwitchCommand(uint16_t sw_id, bool straight) {
        frame.cmdid = 0x0B;

        frame.dlc = 6;
        
        // sw_id is 1-indexed in diagram, but can is 0-indexed. 
        frame.encode_data_0_4(0x3000 + sw_id - 1);

        frame.data[4] = straight;

        frame.data[5] = 1;
    };
};

#endif /* _can_h_ */