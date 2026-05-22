#include <stdint.h>
#include "rpi.h"
#include "time.h"
#include "util.h"
#include "uart.h"

#define TIME_COL "1"
#define TIME_ROW "2"

static char* const TIME_BASE = (char*)(MMIO_BASE + 0x3000);

// TIME register offsets
static const uint32_t TIME_CS   = 0x00;
static const uint32_t TIME_CLO  = 0x04;
static const uint32_t TIME_CHI  = 0x08;

// Masks

uint32_t time_get() {
    return *(volatile uint32_t*)(TIME_BASE + TIME_CLO);
}

const char* format_time(const uint32_t time_us) {
    static char buf[8] = "00:00.0";

    // time is in microseconds
    // so we need to divide by 1M to get S
    // we want 10ths of seconds, so we multiply by 10
    // so end result is we divide by 1M / 10 = 100k
    
    const uint32_t time_ds = time_us / 100000;
    const uint32_t time_s = (time_ds / 10) % 60;
    const uint32_t time_m = time_ds / 10 / 60;
    const uint32_t time_d = time_ds % 10;
    
    // Format MM:SS.D
    buf[0] = '0' + (time_m / 10);
    buf[1] = '0' + (time_m % 10);
    buf[2] = ':';
    buf[3] = '0' + (time_s / 10);
    buf[4] = '0' + (time_s % 10);
    buf[5] = '.';
    buf[6] = '0' + time_d;
    buf[7] = '\0';
    
    return buf;
}

void print_time(uint32_t time_us) {
    uart_puts(CONSOLE, "\033[" TIME_ROW ";" TIME_COL "H");
    uart_puts(CONSOLE, format_time(time_us));
}