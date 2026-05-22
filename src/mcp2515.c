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


// 1000 0abc where abc is txb2, 1, 0 rts
static const uint8_t INSTRUCTION_RTS = 0b10000000;

// 10010nm0, see figure 12-3
static const uint8_t INSTRUCTION_READ_RX = 0b10010000;


// MCP2515 status mask
static const uint8_t STATUS_RX0 = 0x01;
static const uint8_t STATUS_RX1 = 0x02;
static const uint8_t STATUS_TX0 = 0x04;
static const uint8_t STATUS_TX0IF = 0x08;
static const uint8_t STATUS_TX1 = 0x10;
static const uint8_t STATUS_TX1IF = 0x20;
static const uint8_t STATUS_TX2 = 0x40;
static const uint8_t STATUS_TX2IF = 0x80;

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
static const uint8_t TXB0CTRL = 0x30;
static const uint8_t TXB0SIDH = 0x31;

// RX buffer registers
static const uint8_t RXB0CTRL = 0x60;
static const uint8_t RXB1CTRL = 0x70;

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

static void mcp2515_rts(bool txb0 = 0, bool txb1 = 0, bool txb2 = 0) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_RTS | txb0 | (txb1 << 1) | (txb2 << 2));
	spi_end_transaction();
}

// Reads from a receive buffer. Saves on address byte + Clear instruction
static void mcp2515_read_RXn(bool rx0, uint8_t values[], const uint8_t n) {
	spi_begin_transaction();
	spi_transfer_one(INSTRUCTION_READ_RX | (rx0 << 2));

	for (uint8_t i = 0; i < n; i++) {
		values[i] = spi_transfer_one(0x00);
	}

	spi_end_transaction();
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

void mcp2515_send(const CANFRAME& frame) {
	while ((mcp2515_read_status() & STATUS_TX0)) {};


	TXBnFrame mcp_frame;
	memset(&mcp_frame, 0, sizeof(mcp_frame));

	mcp_frame.SIDH = (frame.prio << 4) | (frame.cmdid & 0xF0);

	mcp_frame.SIDL.bits.SID_2_0 = (frame.cmdid & 0b00001110) >> 1;
	mcp_frame.SIDL.bits.EXIDE = 1;
	mcp_frame.SIDL.bits.EID_17_16 = ((frame.cmdid & 0b00000001) << 1) | frame.resp;

	mcp_frame.EID8 = (frame.hash >> 8) & 0xFF;

	mcp_frame.EID0 = frame.hash & 0xFF;

	mcp_frame.DLC.bits.DLC = frame.dlc;
	mcp_frame.DLC.bits.RTR = 0;

	for (int i = 0; i < 8; ++i) {
		mcp_frame.data[i] = frame.data[i];
	}
	mcp2515_write_regs(TXB0SIDH, (const uint8_t*)&mcp_frame.SIDH, sizeof(TXBnFrame) - sizeof(TXBnFrame::CTRL));

	mcp2515_rts(1, 0, 0);
}

bool mcp2515_recieve_RX0(CANFRAME& frame) {
	if (!(mcp2515_read_status() & STATUS_RX0)) return false;

	RXBnFRAME mcp_frame;

	mcp2515_read_RXn(0, (uint8_t*)&mcp_frame.SIDH, sizeof(RXBnFRAME) - sizeof(RXBnFRAME::CTRL));

	frame.prio = (mcp_frame.SIDH & 0xF0) >> 4;
	frame.cmdid = ((mcp_frame.SIDH & 0x0F) << 4) | (mcp_frame.SIDL.bits.SID_2_0 << 1) | ((mcp_frame.SIDL.bits.EID_17_16 & 0b10) >> 1);
	frame.resp = mcp_frame.SIDL.bits.EID_17_16 & 0b1;
	frame.hash = (mcp_frame.EID8 << 8) | mcp_frame.EID0;

	frame.dlc = mcp_frame.DLC.bits.DLC;

	for (int i = 0; i < frame.dlc; ++i) {
		frame.data[i] = mcp_frame.data[i];
	}

	return true;
};

bool mcp2515_recieve_RX1(CANFRAME& frame) {
	if (!(mcp2515_read_status() & STATUS_RX1)) return false;

	RXBnFRAME mcp_frame;

	mcp2515_read_RXn(1, (uint8_t*)&mcp_frame.SIDH, sizeof(RXBnFRAME) - sizeof(RXBnFRAME::CTRL));

	frame.prio = (mcp_frame.SIDH & 0xF0) >> 4;
	frame.cmdid = ((mcp_frame.SIDH & 0x0F) << 4) | (mcp_frame.SIDL.bits.SID_2_0 << 1) | ((mcp_frame.SIDL.bits.EID_17_16 & 0b10) >> 1);
	frame.resp = mcp_frame.SIDL.bits.EID_17_16 & 0b1;
	frame.hash = (mcp_frame.EID8 << 8) | mcp_frame.EID0;

	frame.dlc = mcp_frame.DLC.bits.DLC;

	for (int i = 0; i < frame.dlc; ++i) {
		frame.data[i] = mcp_frame.data[i];
	}

	return true;
};