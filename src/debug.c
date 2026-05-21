#include "debug.h"

#include "uart.h"

static char debug_to_hex_digit(uint8_t x) {
    return x < 10 ? (char)('0' + x) : (char)('A' + (x - 10));
}

static void debug_put_hex8(size_t line, uint8_t value) {
    uart_putc(line, debug_to_hex_digit((uint8_t)((value >> 4) & 0x0F)));
    uart_putc(line, debug_to_hex_digit((uint8_t)(value & 0x0F)));
}

void debug_put_bin8(size_t line, uint8_t value) {
    for (int group = 0; group < 2; ++group) {
        for (int bit = 3; bit >= 0; --bit) {
            uart_putc(line, (value & (1u << (bit + group * 8))) ? '1' : '0');
        }
        uart_putc(line, ' ');
    }
}

void debug_put_bin32(size_t line, uint32_t value) {
    for (int group = 0; group < 8; ++group) {
        for (int bit = 3; bit >= 0; --bit) {
            uart_putc(line, (value & (1u << (bit + group * 8))) ? '1' : '0');
        }
        uart_putc(line, ' ');
    }
}

void debug_clear_console(void) {
    uart_puts(CONSOLE, "\033[3;1H\033[K");
}

void debug_print_memory_dump(const void* start, uint32_t nbytes) {
    const uint8_t* bytes = (const uint8_t*)start;

    uart_puts(CONSOLE, "Memory dump\n\r");
    for (uint32_t row = 0; row < nbytes; row += 16) {
        uart_printf(CONSOLE, "%x: ", (uintptr_t)(bytes + row));
        for (uint32_t i = 0; i < 16; ++i) {
            if (row + i < nbytes) {
                debug_put_hex8(CONSOLE, bytes[row + i]);
            } else {
                uart_puts(CONSOLE, "  ");
            }
        }
        uart_putc(CONSOLE, '\n');
        uart_putc(CONSOLE, '\r');
    }
}

void debug_print_memory_bits(const void* start, uint32_t nbytes) {
    const uint8_t* bytes = (const uint8_t*)start;

    uart_puts(CONSOLE, "Bit inspector\n\r");
    uart_puts(CONSOLE, "addr.  bits       hex\n\r");
    uart_puts(CONSOLE, "-----  ---------  ----\n\r");
    for (uint32_t i = 0; i < nbytes; ++i) {
        uart_printf(CONSOLE, "%x  ", (uintptr_t)(bytes + i));
        debug_put_bin8(CONSOLE, bytes[i]);
        uart_printf(CONSOLE, " 0x%x\n\r", bytes[i]);
    }
}