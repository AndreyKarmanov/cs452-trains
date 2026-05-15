#ifndef _mcp2515_h_
#define _mcp2515_h_ 1

#include <stdint.h>

/** Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI. */
void mcp2515_init(void);

/** Report and drop a frame if RX0 is full. */
int mcp2515_fakerecv();

#endif /* _mcp2515_h_ */
