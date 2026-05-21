#include "mcp2515.h"
#include "debug.h"
#include "spi.h"
#include "can.h"
#include "uart.h"

// configuration registers
static const uint8_t CNF3 = 0x28;
static const uint8_t CNF2 = 0x29;
static const uint8_t CNF1 = 0x2A;

// MCP2515 configuration for 16 MHz clock and 250 kbit/s bitrate
// Chapter 5.0 Bit Timing in MCP2515 datasheet
// and/or https://kvaser.com/support/calculators/bit-timing-calculator/.
static const uint8_t MCP_16MHz_250kbPS_CFG1 = 0x41;
static const uint8_t MCP_16MHz_250kbPS_CFG2 = 0xF1;
static const uint8_t MCP_16MHz_250kbPS_CFG3 = 0x85;

// MCP2515 normal operation mode
static const uint8_t OPMODE_NORMAL = 0x00;

// MCP2515 instruction set
static const uint8_t INSTRUCTION_WRITE = 0x02;
static const uint8_t INSTRUCTION_READ = 0x03;
static const uint8_t INSTRUCTION_BIT_MODIFY = 0x05;
static const uint8_t INSTRUCTION_READ_STATUS = 0xA0;

// MCP2515 status mask
static const uint8_t STATUS_RX0 = 0x01;

// MCP2515 buffer registers
static const uint8_t RXBnCTRL0 = 0x60;
static const uint8_t RXBnCTRL1 = 0x70;

// control and status registers
static const uint8_t CANSTAT = 0x0E;
static const uint8_t CANCTRL = 0x0F;
// OPMOD mask for CANSTAT register
static const uint8_t CANSTAT_OPMOD = 0xE0;
// REQOP mask for CANCTRL register
static const uint8_t CANCTRL_REQOP = 0xE0;

// flags register
static const uint8_t CANINTF = 0x2C;

// TX buffer registers
// set bit 3 to 1 to send, check if 0 before writing
static const uint8_t TXB0CTRL = 0x30;
static const uint8_t TXB0SIDH = 0x31;
static const uint8_t TXB0DLC = 0x35;
static const uint8_t TXB0D0 = 0x36;

static const uint8_t TXB1DLC = 0x45;
static const uint8_t TXB1D0 = 0x46;

static const uint8_t TXB2DLC = 0x55;
static const uint8_t TXB2D0 = 0x56;
static const uint64_t t = 0x3D;

/** Read n consecutive registers starting from the specified one. */
static void mcp2515_read_regs(uint8_t reg, uint8_t values[], const uint8_t n) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_READ);
	spi_transfer_one(reg);
	for (uint8_t i = 0; i < n; i++) {
		// during transaction, address pointer is automatically incremented after each byte transfer.
		values[i] = spi_transfer_one(0x00);
	}
	spi_end_transaction();
}

/** Read the value of a single register. */
static uint8_t mcp2515_read_reg(uint8_t reg) {
	uint8_t ret = 0;
	mcp2515_read_regs(reg, &ret, 1);
	return ret;
}

/** Write values to n consecutive registers starting from the specified one. */
static void mcp2515_write_regs(uint8_t reg, const uint8_t values[], const uint8_t n) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_WRITE);
	spi_transfer_one(reg);
	for (uint8_t i = 0; i < n; i++) {
		spi_transfer_one(values[i]);
	}
	spi_end_transaction();
}

/** Write a value to a single register. */
static void mcp2515_write_reg(uint8_t reg, const uint8_t value) {
	mcp2515_write_regs(reg, &value, 1);
}

/** Modify individual bits of a register according to mask. */
static void mcp2515_modify_reg(uint8_t reg, const uint8_t mask, const uint8_t data) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_BIT_MODIFY);
	spi_transfer_one(reg);
	spi_transfer_one(mask);
	spi_transfer_one(data);
	spi_end_transaction();
}

/** Read the status of the MCP2515, including RX and TX buffers. */
static uint8_t mcp2515_read_status(void) {
	uint8_t ret = 0;
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_READ_STATUS);
	ret = spi_transfer_one(0x00);
	spi_end_transaction();
	return ret;
}

void mcp2515_init(void) {
	// No need to reset MCP2515 here as a hardware reset is done during boot.
	// MCP2515 automatically enters config mode after hardware reset.

	// Set the bitrate configuration registers
	mcp2515_write_reg(CNF1, MCP_16MHz_250kbPS_CFG1);
	mcp2515_write_reg(CNF2, MCP_16MHz_250kbPS_CFG2);
	mcp2515_write_reg(CNF3, MCP_16MHz_250kbPS_CFG3);

	// do not filter messages. Allow rollover of RXB0 to RXB1.
	mcp2515_write_reg(RXBnCTRL0, 0x64);
	mcp2515_write_reg(RXBnCTRL1, 0x60);

	// start MCP2515 by setting operation mode to normal
	mcp2515_modify_reg(CANCTRL, CANCTRL_REQOP, OPMODE_NORMAL);
	while ((mcp2515_read_reg(CANSTAT) & CANSTAT_OPMOD) != OPMODE_NORMAL); // wait until mode is set
}

int mcp2515_fakerecv() {
	if (!(mcp2515_read_status() & STATUS_RX0)) return 0;
	mcp2515_write_reg(CANINTF, 0); // fake confirm receipt of frame
	return 1;
}

void* memset(void* s, int c, size_t n) {
	for (char* it = (char*)s; n > 0; --n) *it++ = c;
	return s;
}

void mcp2515_send(const CANFRAME* frame) {
	TXBnFrame mcp_frame;
	memset(&mcp_frame, 0, sizeof(mcp_frame));

	debug_put_bin32(CONSOLE, (frame->prio << 4) | (frame->cmdid & 0xF0));
	mcp_frame.sid.H = (frame->prio << 4) | (frame->cmdid & 0xF0); // SID[10:3]

	uart_puts(CONSOLE, "\n\rSIDL\n\r");
	mcp_frame.sid.L = ((frame->cmdid & 0b00001110) << 4) | (1 << 3) | ((frame->cmdid & 0b00000001) << 1) | frame->resp; // SID[2:0] in bits 7:5, EXIDE in bit 3, EID[17:16] in bits 1:0
	debug_put_bin8(CONSOLE, mcp_frame.sid.L);

	uart_puts(CONSOLE, "\n\rEID8\n\r");
	mcp_frame.eid.EID8 = (frame->hash >> 8) & 0xFF; // EID[15:8]
	debug_put_bin8(CONSOLE, mcp_frame.eid.EID8);

	uart_puts(CONSOLE, "\n\rEID0\n\r");
	mcp_frame.eid.EID0 = frame->hash & 0xFF; // EID[7:0]
	debug_put_bin8(CONSOLE, mcp_frame.eid.EID0);

	uart_puts(CONSOLE, "\n\rDLC\n\r");
	mcp_frame.dlc.DLC = frame->dlc; // DLC is 4 bits
	mcp_frame.dlc.RTR = 0; // Data frame
	debug_put_bin8(CONSOLE, *(uint8_t*)&mcp_frame.dlc);

	for (int i = 0; i < 8; ++i) {
		mcp_frame.data[i] = frame->data[i];
	}

	
	uart_puts(CONSOLE, "TXBnFrame bit view:\n\r");
	debug_print_memory_bits(&mcp_frame, sizeof(mcp_frame));
	mcp2515_write_regs(TXB0SIDH, (const uint8_t*)&mcp_frame.sid, sizeof(TXBnFrame) - 1);

	mcp_frame.ctrl.bits.TXREQ = 1;
	mcp2515_write_reg(TXB0CTRL, mcp_frame.ctrl.byte);
}