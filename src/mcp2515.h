#ifndef _mcp2515_h_
#define _mcp2515_h_ 1

#include <stdint.h>
#include "can.h"

// frame is structed in same way the buffer is laid out
// this relies on the memory being laid out in the same way
// this can potentially cause issues if flags change, compiler changes, hardware, etc.
struct TXBnFrame {

    // union
    // {
    //     // we write the TXBnCTRL register separately, so we need a byte-level view
    //     // otherwise we just write the full TXBnFrame starting from SIDH to DATA8
    //     uint8_t byte;
    //     struct {
    //         uint8_t TXP0 : 1;
    //         uint8_t TXP1 : 1;
    //         uint8_t : 1;
    //         uint8_t TXREQ : 1;
    //         uint8_t TXERR : 1;
    //         uint8_t MLOA : 1;
    //         uint8_t ABTF : 1;
    //         uint8_t : 1;
    //     } bits;
    // } CTRL;

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

    TXBnFrame() = default;

    TXBnFrame(const CANFRAME frame) : SIDH((frame.prio << 4) | (frame.cmdid & 0xF0)),
        EID8((frame.hash >> 8) & 0xFF), EID0(frame.hash & 0xFF) {


        SIDL.bits.SID_2_0 = (frame.cmdid & 0b00001110) >> 1;
        SIDL.bits.EXIDE = 1;
        SIDL.bits.EID_17_16 = ((frame.cmdid & 0b00000001) << 1) | frame.resp;

        DLC.bits.DLC = frame.dlc;
        DLC.bits.RTR = 0;

        __builtin_memcpy(data, frame.data, frame.dlc);
    }
};

struct RXBnFRAME {

    // the ctrl are diff for the two buffers
    // we just need to make sure RXM is set to 0b11 and BUKT is 1
    // this is to remove filters and ensure rollover is enabled
    // union
    // {
    //     uint8_t byte;
    //     struct {
    //         uint8_t : 2;
    //         uint8_t BUKT : 1; // should set/read 1 every time
    //         uint8_t RXRTR : 1;
    //         uint8_t : 1;
    //         uint8_t RXM : 2; // should set/read 0b11 always
    //         uint8_t : 1;
    //     } bits;
    // } CTRL;

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


/** Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI. */
void mcp2515_init(void);

/** Report and drop a frame if RX0 is full. */
int mcp2515_fakerecv();

bool mcp2515_recieve_RXn(bool rx0, CANFRAME& frame);

void mcp2515_send(const TXBnFrame frame);
void mcp2515_send(const CANFRAME frame, uint32_t delay_us);
void mcp2515_send_pending();


#endif /* _mcp2515_h_ */
