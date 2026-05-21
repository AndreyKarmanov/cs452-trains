#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "rpi.h"
#include "mcp2515.h"
#include "debug.h"
#include "uart.h"
#include "time.h"
#include "can.h"
#include <ctype.h>

extern "C" void setup_mmu(); // in mmu.S

#define CONSOLE_ROW "3"
#define CLOCK_UPDATE_INTERVAL_US 100000

typedef enum COMMAND_T {
	COMMAND_NONE,
	COMMAND_MOVE,
	COMMAND_QUIT
} COMMAND_T;

const int x = 0x3D - 0x36;

static COMMAND_T parse_command(const char* buf, size_t blen) {
	if (blen == 0) {
		return COMMAND_NONE;
	}
	if (strcmp(buf, "Q") == 0 || strcmp(buf, "q") == 0) {
		return COMMAND_QUIT;
	}
	if (strcmp(buf, "MOVE") == 0) {
		return COMMAND_MOVE;
	}
	return COMMAND_NONE;
}


extern "C" int kmain() {
#if defined(MMU)
	setup_mmu();
#endif


	// set up GPIO pins for both console uart and canbus
	gpio_init();
	// set up CANbus controller
	mcp2515_init();
	// not strictly necessary, since console is configured during boot
	uart_config_and_enable(CONSOLE);

	// Clear, center
	uart_puts(CONSOLE, "\033[2J\033[H");
	uart_puts(CONSOLE, __DATE__ " / " __TIME__ " / Andrey Karmanov ");

	SpeedCommand sample(15, 100);

	CANFRAME frame = sample.frame;

	uart_printf(CONSOLE, "Initial frame data[0]: %u\n\r", sizeof(TXBnFrame));
	uart_puts(CONSOLE, "Raw CANFRAME bytes:\n\r");
	debug_print_memory_dump(&frame, sizeof(frame));
	uart_puts(CONSOLE, "Raw CANFRAME bits:\n\r");
	debug_print_memory_bits(&frame, sizeof(frame));
	mcp2515_send(&frame);

	uint32_t cmd_buf_n = 0;
	char cmd_buf[32];


	for (;;) {

		// handle user input in a timely manner, i.e. if they have a number of bytes we pull all at once
		for (;;) {

			// try to fetch a byte
			char c = uart_maybec(CONSOLE);
			if (c == 0) {
				break;
			}

			// check if it's a printable character (i.e. a char used in a command)
			if (isprint(c)) {
				if (cmd_buf_n < 30) {
					cmd_buf[cmd_buf_n] = c;
					uart_putc(CONSOLE, c);
					++cmd_buf_n;
				}
			} else if ((c == 0x08 || c == 0x7f) && cmd_buf_n > 0) { // backspace
				uart_puts(CONSOLE, "\b \b"); // move back, print space, move back again
				--cmd_buf_n;
			} else if (c == '\r') { // enter
				debug_clear_console();
				cmd_buf[cmd_buf_n] = '\0';
				COMMAND_T cmd = parse_command(cmd_buf, cmd_buf_n);
				cmd_buf_n = 0;
				if (cmd == COMMAND_QUIT) {
					uart_puts(CONSOLE, "Goodbye!\n\r");
					return 0;
				} else if (cmd == COMMAND_MOVE) {
					SpeedCommand cmd(15, 500);
					mcp2515_send(&cmd.frame);
				} else {
					uart_puts(CONSOLE, "Unknown command\n\r");
				}
			}
		}
		if (mcp2515_fakerecv()) {
			uart_puts(CONSOLE, "FRAME\n\r");
		}

		// // update clock
		// uint32_t new_time = time_get();
		// if (new_time - time > CLOCK_UPDATE_INTERVAL_US) {
		// 	time = new_time;
		// 	print_time(time);
		// }

		// // reset the cursor
		// uart_printf(CONSOLE, "\033[" CONSOLE_ROW ";%uH", 1 + cmd_buf_n);
	}
}

#if !defined(MMU)
#include <stddef.h>

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void* s, int c, size_t n) {
	for (char* it = (char*)s; n > 0; --n) *it++ = c;
	return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void* memcpy(void* dest, const void* src, size_t n) {
	char* sit = (char*)src;
	char* cdest = (char*)dest;
	for (size_t i = 0; i < n; ++i) *cdest++ = *sit++;
	return dest;
}
#endif
