#include <string.h>
#include <stdint.h>
#include "rpi.h"
#include "mcp2515.h"
#include "uart.h"
#include "time.h"

extern void setup_mmu(); // in mmu.S

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
	// welcome message
	uart_puts(CONSOLE, "\r\nHello world, this is version: " __DATE__ " / " __TIME__ "\r\n\r\nPremakess 'q' to reboot\r\n");

	uint32_t time = time_get();  // tenths digit
	for (;;) {
		for (;;) {
			char c = uart_maybec(CONSOLE);
			if (c == 0) {
				break;
			}
			uart_putc(CONSOLE, c);
			if (c == '\r') {
				uart_putc(CONSOLE, '\n');
				break;
			} else if (c == 'q' || c == 'Q') {
				uart_puts(CONSOLE, "\r\n");
				return 0;
			} else if (c == 'c') {
				uart_puts(CONSOLE, "\033[2J\033[H");
				break;
			} else if (c == '1') {
				uart_puts(CONSOLE, "\033(0\r\n");
				break;
			} else if (c == '0') {
				uart_puts(CONSOLE, "\033(B\r\n");
				break;
			} 
		}
		if (mcp2515_fakerecv()) {
			uart_puts(CONSOLE, "FRAME\n\r");
		}

		// update clock
		uint32_t new_time = time_get();
		if (new_time - time > 100000) {
			time = new_time;
			uart_puts(CONSOLE, "\033[2J\033[H");
			uart_puts(CONSOLE, format_time(time));
		}
	}
}

#if !defined(MMU)
#include <stddef.h>

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void *s, int c, size_t n) {
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
