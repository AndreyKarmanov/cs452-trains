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
#define UART_FLUSH_BUDGET_US 5000

static void print_debug(uint32_t loop_time_us, uint32_t draws) {
	uart_printf(CONSOLE, "\033[3;1HLoop: %u us (%u ms)  Draws: %u  UART dropped: %u    ",
		loop_time_us, loop_time_us / 1000, draws, uart_tx_dropped(CONSOLE));
}

typedef struct TaskContext	
{
	uint64_t x19;
	uint64_t x20;
	uint64_t x21;
	uint64_t x22;
	uint64_t x23;
	uint64_t x24;
	uint64_t x25;
	uint64_t x26;
	uint64_t x27;
	uint64_t x28;
	uint64_t x29; // frame pointer
	uint64_t x30; // link register
};

TaskContext task_contexts[12];
uint8_t task_stacks[12][4096] __attribute__((aligned(16)));

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

	uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__ " / Andrey Karmanov\n\r");
	CANFRAME frame;
	uint32_t time = 0;
	State state;
	
	print_state(state, true);
	apply_state(state);
	clear_console();


	uint32_t last_loop_time = 0;
	uint32_t draws = 0;
	for (;;) {
		uint32_t start = time_get();

		auto cmd = update_console(state);

		if (cmd == COMMAND_T::COMMAND_QUIT) {
			uart_puts(CONSOLE, "\033[2J\033[HGoodbye!");
			break;
		} else if (cmd == COMMAND_T::COMMAND_REDRAW || uart_tx_dropped(CONSOLE) > 10'000) {
			clear_uart_dropped(CONSOLE);
		}

		if (mcp2515_recieve_RXn(0, frame)) {
			state.update_from_mrk(decode_frame(frame));
		}

		if (mcp2515_recieve_RXn(1, frame)) {
			state.update_from_mrk(decode_frame(frame));
		}

		uint32_t new_time = time_get();
		draws += print_state(state, cmd == COMMAND_T::COMMAND_REDRAW);
		if (new_time - time > CLOCK_UPDATE_INTERVAL_US) {
			time = new_time;
			print_time(time);
			print_debug(last_loop_time, draws);
		}
		mcp2515_send_pending();
		uart_flush(CONSOLE, UART_FLUSH_BUDGET_US);

		last_loop_time = time_get() - start;
	}
	uart_puts(CONSOLE, "\033[?25h");
	uart_flush_all(CONSOLE);

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
