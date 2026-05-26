#include <string.h>
#include "rpi.h"
#include "mcp2515.h"
#include "uart.h"

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
	uart_puts(CONSOLE, "\r\nHello world, this is version: " __DATE__ " / " __TIME__ "\r\n\r\nPress 'q' to reboot\r\n");

	unsigned int counter = 1;
	for (;;) {
		uart_printf(CONSOLE, "PI[%u]> ", counter++);
		for (;;) {
			char c = uart_getc(CONSOLE);
			uart_putc(CONSOLE, c);
			if (c == '\r') {
				uart_putc(CONSOLE, '\n');
				break;
			} else if (c == 'q' || c == 'Q') {
				uart_puts(CONSOLE, "\r\n");
				return 0;
			}
		}
		if (mcp2515_fakerecv()) {
			uart_puts(CONSOLE, "FRAME\n\r");
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
