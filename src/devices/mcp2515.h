#pragma once
#include "mrk.h"
#include <cstring>
#include <stdint.h>

// frame is structed in same way the buffer is laid out
// this relies on the memory being laid out in the same way
// this can potentially cause issues if flags change, compiler changes,
// hardware, etc. packed attribute is to ensure that compiler keeps 0 extra bits
// or bytes betwene fields since we are copying directly inoto buffers based on
// this frame, need to keep it!
struct __attribute__((packed)) TXBnFrame {

  uint8_t SIDH; // SID[10:3]

  union {
    uint8_t byte;
    struct {
      uint8_t EID_17_16 : 2;
      uint8_t : 1;
      uint8_t EXIDE : 1;
      uint8_t : 1;
      uint8_t SID_2_0 : 3;
    } bits;
  } SIDL;

  uint8_t EID8; // EID[15:8]
  uint8_t EID0; // EID[7:0]

  union {
    uint8_t byte;
    struct {
      uint8_t DLC : 4;
      uint8_t : 2;
      uint8_t RTR : 1;
      uint8_t : 1;
    } bits;
  } DLC;

  uint8_t data[8];

  TXBnFrame() = default;

  // need these ugly casts because the compiler complains (if we include any
  // 0x0X) it makes it int an int, then complains about narrowing don't add
  // explicit here because we want the CANFRAME to be easily convertible to send
  TXBnFrame(const CANFRAME frame)
      : SIDH(uint8_t((frame.prio << 4) | (frame.cmdid & 0xF0))),
        SIDL{.byte =
                 uint8_t(((frame.cmdid & 0x0E) << 4) | 0x08 |
                         ((frame.cmdid & 0x01) << 1) | (frame.resp & 0x01))},
        EID8(uint8_t((frame.hash >> 8) & 0xFF)),
        EID0(uint8_t(frame.hash & 0xFF)),
        DLC{.byte = uint8_t(frame.dlc & 0x0F)} {
    std::memcpy(data, frame.data, 8);
  }
};

struct __attribute__((packed)) RXBnFRAME {

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

  uint8_t SIDH; // SID[10:3]

  union {
    uint8_t byte;
    struct {
      uint8_t EID_17_16 : 2;
      uint8_t : 1;
      uint8_t IDE : 1; // EXIDE equiv, should be 1 always
      uint8_t SRR : 1;
      uint8_t SID_2_0 : 3;
    } bits;
  } SIDL;

  uint8_t EID8; // EID[15:8]
  uint8_t EID0; // EID[7:0]

  union {
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

/** Initialize the MCP2515 CAN controller. Should be called after initializing
 * GPIO and SPI. */
void mcp2515_init(void);

/** Report and drop a frame if RX0 is full. */
int mcp2515_fakerecv();

bool mcp2515_recieve(CANFRAME &frame);
bool mcp2515_recieve_RXn(bool rx0, CANFRAME &frame);

bool mcp2515_send(const TXBnFrame frame);
bool mcp2515_tx_ready();
bool mcp2515_rx_pending();

struct CANINT {
  bool merre : 1  = false;
  bool wakie : 1  = false;
  bool errie : 1  = false;
  bool tx2ie : 1  = false;
  bool tx1ie : 1  = false;
  bool tx0ie : 1  = false;
  bool rxi1e : 1  = false;
  bool rxi0ie : 1 = false;

  // so we can static_cast to uint8_t
  explicit operator uint8_t() const noexcept {
    return uint8_t((merre ? 0x80 : 0) | (wakie ? 0x40 : 0) |
                   (errie ? 0x20 : 0) | (tx2ie ? 0x10 : 0) |
                   (tx1ie ? 0x08 : 0) | (tx0ie ? 0x04 : 0) |
                   (rxi1e ? 0x02 : 0) | (rxi0ie ? 0x01 : 0));
  }

  // can't make this a constructor as
  // CANINT{.rxi0ie = true, .rxi1ie = true} won't wokr
  static constexpr CANINT from_byte(uint8_t b) noexcept {
    return CANINT{.merre  = (b & 0x80) != 0,
                  .wakie  = (b & 0x40) != 0,
                  .errie  = (b & 0x20) != 0,
                  .tx2ie  = (b & 0x10) != 0,
                  .tx1ie  = (b & 0x08) != 0,
                  .tx0ie  = (b & 0x04) != 0,
                  .rxi1e  = (b & 0x02) != 0,
                  .rxi0ie = (b & 0x01) != 0};
  }
};

void enable_mcp2515_interrupt(const CANINT &interrupts);

void disable_mcp2515_interrupt(const CANINT &interrupts);

CANINT mcp2515_get_enabled_interrupt();

// all active interrupts
CANINT mcp2515_get_active_irq();
