#include <stdint.h>
#include <variant>

constexpr uint32_t MRK_HASH = 0xC3;

struct TXBnCTRL {
    uint8_t TXP0 : 1 = 0; // default to lowest prio
    uint8_t TXP1 : 1 = 0;
    uint8_t : 1;
    uint8_t TXREQ : 1 = 1; // default to sendable
    uint8_t TXERR : 1;
    uint8_t MLOA : 1;
    uint8_t ABTF : 1;
    uint8_t : 1;
};

struct TXBnSID {
    uint8_t SIDH;   // SID[10:3]
    uint8_t SIDL;   // SID[2:0] | EXIDE | EID[17:16]
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
    TXBnCTRL   ctrl;
    TXBnSID    sid;
    TXBnEID    eid;
    TXBnDLC    dlc;
    uint8_t    data[8];
};


struct CANFRAME {
    union {
        uint32_t raw;

        struct
        {
            uint32_t prio : 4;
            uint32_t cmdid : 8;
            uint32_t resp : 1;
            uint32_t hash : 16;
        } b;
    } msgid;

    uint16_t dlc : 4;
    uint8_t data[8];
};

struct LightCommand
{
    CANFRAME frame{
        frame.msgid.b.cmdid = 0x06,
        frame.msgid.b.prio = 0,
        frame.msgid.b.resp = 0,
        frame.msgid.b.hash = MRK_HASH,
        frame.dlc = 6,
        frame.data[4] = 0
    };

    LightCommand(uint32_t loco_id, bool value) {
        frame.data[4] = 0;
        frame.data[5] = value;

        frame.data[0] = loco_id & 0xFF;
        frame.data[1] = (loco_id >> 8) & 0xFF;
        frame.data[2] = (loco_id >> 16) & 0xFF;
        frame.data[3] = (loco_id >> 24) & 0xFF;
    };
};



// flow is RXBnFrame <-> MRKCommand <-> TXBnFrame
// this is specialized locmotive funciton?

// struct LightCommand {
//     uint8_t cmdid = 0x06; // Lok Funktion mention page 32 on MRK
//     uint8_t dlc = 0x06; // Aktivieren einer Funktion (lol)
//     uint32_t loco_id = 0; // DATA[0:3]
//     uint8_t func = 0; // DATA[4], 0 is light
//     uint8_t on = 1; // DATA[5], 1/0 for on /off
// };