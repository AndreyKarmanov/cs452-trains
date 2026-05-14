#include <string.h>
#include <stdint.h>
#include "rpi.h"
#include "mcp2515.h"
#include "uart.h"
#include "time.h"
#include <ctype.h>

extern void setup_mmu(); // in mmu.S

#define CONSOLE_ROW "3"
#define CLOCK_UPDATE_INTERVAL_US 100000

void uart_clear_console() {
	uart_puts(CONSOLE, "\033[" CONSOLE_ROW ";1H\033[K");
}

typedef enum COMMAND_T {
	COMMAND_NONE,
	COMMAND_QUIT
} COMMAND_T;


COMMAND_T parse_command(const char* buf, size_t blen) {
	if (blen == 0) {
		return COMMAND_NONE;
	}
	if (strcmp(buf, "Q") == 0 || strcmp(buf, "q") == 0) {
		return COMMAND_QUIT;
	}
	return COMMAND_NONE;
}

int kmain() {
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
	uart_puts(CONSOLE, __DATE__ " / " __TIME__ " / Andrey Karmanov\r\n");

	uint32_t time = time_get();  // tenths digit.

	uint32_t chars = 0;

	char buf[32];

	for (;;) {
		for (;;) {
			char c = uart_maybec(CONSOLE);
			if (c == 0) {
				break;
			}
			if (isprint(c)) {
				if (chars < 30) {
					buf[chars] = c;
					uart_putc(CONSOLE, c);
					++chars;
				}
			} else if ((c == 0x08 || c == 0x7f) && chars > 0) {
				uart_puts(CONSOLE, "\b \b");
				--chars;
			} else if (c == '\r') {
				uart_clear_console();
				buf[chars] = '\0';
				COMMAND_T cmd = parse_command(buf, chars);
				chars = 0;
				if (cmd == COMMAND_QUIT) {
					uart_puts(CONSOLE, "Goodbye!\n\r");
					return 0;
				}
			}
		}
		if (mcp2515_fakerecv()) {
			uart_puts(CONSOLE, "FRAME\n\r");
		}

		// update clock
		uint32_t new_time = time_get();
		if (new_time - time > CLOCK_UPDATE_INTERVAL_US) {
			time = new_time;
			print_time(time);
		}

		// reset the cursor
		uart_printf(CONSOLE, "\033[" CONSOLE_ROW ";%uH", 1 + chars);
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
