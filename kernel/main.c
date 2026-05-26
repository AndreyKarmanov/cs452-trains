#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "rpi.h"
#include "uart.h"

extern "C" void setup_mmu(); // in mmu.S

extern "C" int kmain() {
#if defined(MMU)
	setup_mmu();
#endif
	// set up GPIO pins for both console uart and canbus
	gpio_init();
	// not strictly necessary, since console is configured during boot
	uart_config_and_enable(CONSOLE);

	uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__ " / Andrey Karmanov / Anthony Ho\n\r");

	for (;;) {

	}
	return 0;
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
