#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "rpi.h"
#include "mcp2515.h"
#include "debug.h"
#include "uart.h"
#include "can.h"
#include "state.h"
#include "console.h"
#include "buffer.h"
#include "time.h"

extern "C" void setup_mmu(); // in mmu.S

#define CLOCK_UPDATE_INTERVAL_US 100000

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
	uart_puts(CONSOLE, "\033[2J\033[H" __DATE__ " / " __TIME__ " / Andrey Karmanov ");

	CANFRAME frame;
	uint32_t time = 0;
	State state;
	print_state(state, 1);
	clear_console();

	uart_puts(CONSOLE, "\033[?25l");

	for (;;) {

		auto cmd = update_console();

		if (cmd == COMMAND_T::COMMAND_QUIT) {
			uart_puts(CONSOLE, "\033[2J\033[HGoodbye!");
			break;
		}

		if (mcp2515_recieve_RXn(0, frame)) {
			state.update_from_mrk(decode_frame(frame));
		}

		if (mcp2515_recieve_RXn(1, frame)) {
			state.update_from_mrk(decode_frame(frame));
		}

		uint32_t new_time = time_get();
		if (new_time - time > CLOCK_UPDATE_INTERVAL_US) {
			time = new_time;
			print_time(time);
		}
		
		print_state(state);
	}
	uart_puts(CONSOLE, "\033[?25h");

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
