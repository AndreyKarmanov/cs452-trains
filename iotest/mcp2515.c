#include "mcp2515.h"
#include "spi.h"

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
static const uint8_t INSTRUCTION_WRITE       = 0x02;
static const uint8_t INSTRUCTION_READ        = 0x03;
static const uint8_t INSTRUCTION_BIT_MODIFY  = 0x05;
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
