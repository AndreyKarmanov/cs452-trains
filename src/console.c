#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <ctype.h>

#include "console.h"
#include "uart.h"
#include "can.h"
#include "mcp2515.h"

#define CONSOLE_ROW_START "3"
#define CONSOLE_ROW_TERM "4"
#define CONSOLE_ROW_HIST "5"

void clear_console(void) {
    uart_puts(CONSOLE, "\033[" CONSOLE_ROW_START ";1H\033[KConsole\n\r");
    uart_puts(CONSOLE, "\033[" CONSOLE_ROW_TERM ";1H\033[K> ");
}

// parse and fire command 
static COMMAND_T fire_command(const char* buf, size_t blen) {
    if (blen == 0) return COMMAND_NONE;

    size_t pos = 0;

    // skip leading spaces
    while (pos < blen && isblank(buf[pos])) pos++;
    if (pos == blen) return COMMAND_NONE;

    size_t cmd_start = pos;
    while (pos < blen && !isblank(buf[pos])) pos++;
    size_t cmd_len = pos - cmd_start;

    auto expect_int = [&]() -> int32_t {
        while (pos < blen && isblank(buf[pos])) pos++;
        if (pos == blen || !isdigit(buf[pos])) return -1;
        uint32_t val = 0;
        while (pos < blen && isdigit(buf[pos])) {
            val = val * 10 + (buf[pos] - '0');
            pos++;
        }
        return val;
    };

    auto expect_end = [&]() -> bool {
        while (pos < blen && isblank(buf[pos])) pos++;
        return pos == blen;
    };

    if (cmd_len == 1 && (buf[cmd_start] == 'q' || buf[cmd_start] == 'Q') && expect_end()) {
        return COMMAND_QUIT;
    }

    if (cmd_len == 2 && strncmp(buf + cmd_start, "tr", 2) == 0) {
        int32_t loco_id = expect_int();
        int32_t speed = expect_int();
        if (loco_id >= 0 && speed >= 0 && expect_end()) {
            mcp2515_send(SpeedCommand(loco_id, speed).to_frame());
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: tr %u %u", buf, loco_id, speed);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is tr <train number> <train speed>", buf);
        }
        return COMMAND_NONE;
    }

    if (cmd_len == 2 && strncmp(buf + cmd_start, "sw", 2) == 0) {
        int32_t sw_id = expect_int();
        // expect 'C' or 'S'
        while (pos < blen && isblank(buf[pos])) pos++;
        if (sw_id >= 0 && pos < blen) {
            char dir = buf[pos];
            pos++; // consume 'C' or 'S'
            if ((dir == 'S' || dir == 'C' || dir == 's' || dir == 'c') && expect_end()) {
                bool is_straight = (dir == 'S' || dir == 's');
                mcp2515_send(SwitchCommand(sw_id, is_straight).to_frame());
                uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: sw %u %c", buf, sw_id, is_straight ? 'S' : 'C');
                return COMMAND_NONE;
            }
        }
        uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is sw <switch number> <switch direction>", buf);
        return COMMAND_NONE;
    }

    if (cmd_len == 2 && strncmp(buf + cmd_start, "rv", 2) == 0) {
        int32_t loco_id = expect_int();
        if (loco_id >= 0 && expect_end()) {
            mcp2515_send(SpeedCommand(loco_id, 0).to_frame());
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: rv %u (stopped)", buf, loco_id);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is rv <train number>", buf);
        }
        return COMMAND_NONE;
    }

    uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Unknown command. Available: q, tr, sw, rv", buf);
    return COMMAND_NONE;
}

static char cmd_buf[32];
static uint32_t cmd_buf_n = 0;

COMMAND_T update_console() {
    COMMAND_T cmd = COMMAND_NONE;

    char c = uart_maybec(CONSOLE);
    if (c) {
        uart_printf(CONSOLE, "\033[" CONSOLE_ROW_TERM ";%uH", 3 + cmd_buf_n);
    }
    while (c) {
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
            clear_console();
            cmd_buf[cmd_buf_n] = '\0';
            cmd = fire_command(cmd_buf, cmd_buf_n);
            cmd_buf_n = 0;
            break;
        }
        c = uart_maybec(CONSOLE);
    }

    return cmd;
}