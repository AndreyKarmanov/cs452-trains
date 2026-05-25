#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <ctype.h>

#include "console.h"
#include "state.h"
#include "uart.h"
#include "can.h"
#include "mcp2515.h"
#include "time.h"
#include "state.h"

#define CONSOLE_ROW_START "4"
#define CONSOLE_ROW_TERM "5"
#define CONSOLE_ROW_HIST "6"

void clear_console(void) {
    uart_puts(CONSOLE, "\033[" CONSOLE_ROW_START ";1H\033[KConsole\n\r");
    uart_puts(CONSOLE, "\033[" CONSOLE_ROW_TERM ";1H\033[K> ");
}

// parse and fire command 
static COMMAND_T fire_command(const char* buf, size_t blen, State& state) {
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
            state.command_timings_start[2] = time_get();
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: tr %u %u", buf, loco_id, speed);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is tr <train number> <train speed>", buf);
        }
        return COMMAND_NONE;
    }

    if (cmd_len == 2 && strncmp(buf + cmd_start, "lr", 2) == 0) {
        int32_t loco_id = expect_int();
        int32_t light = expect_int();
        if (loco_id >= 0 && light >= 0 && expect_end()) {
            mcp2515_send(LightCommand(loco_id, light).to_frame());
            state.command_timings_start[1] = time_get();
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: lr %u %u", buf, loco_id, light);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is lr <train number> <light state>", buf);
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
                state.command_timings_start[4] = time_get();
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
            mcp2515_send(DirectionCommand(loco_id, false).to_frame(), TIME_1S_US * 10);
            state.command_timings_start[3] = time_get();
            mcp2515_send(SpeedCommand(loco_id, state.get_loco(loco_id).requested_speed).to_frame(), TIME_1S_US * 11);

            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: rv %u (stopping)", buf, loco_id);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is rv <train number>", buf);
        }
        return COMMAND_NONE;
    }


    if (cmd_len == 4 && strncmp(buf + cmd_start, "stop", 4) == 0) {
        if (expect_end()) {
            mcp2515_send(ControlCommand(ControlCommand::CMD_STOP).to_frame());
            state.command_timings_start[6] = time_get();
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: stop (stopping)", buf);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is stop", buf);
        }
        return COMMAND_NONE;
    }

    if (cmd_len == 2 && strncmp(buf + cmd_start, "go", 2) == 0) {
        if (expect_end()) {
            mcp2515_send(ControlCommand(ControlCommand::CMD_GO).to_frame());
            state.command_timings_start[6] = time_get();
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: go (starting)", buf);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is go", buf);
        }
        return COMMAND_NONE;
    }

    if (cmd_len == 5 && strncmp(buf + cmd_start, "reset", 5) == 0) {
        if (expect_end()) {
            apply_state(State{});
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Success: reset (resetting all state)", buf);
        } else {
            uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Format is reset", buf);
        }
        return COMMAND_NONE;
    }

    uart_printf(CONSOLE, "\033[" CONSOLE_ROW_HIST ";1H\033[K> %s\n\r\033[K  Error: Unknown command. Available: q, tr, sw, rv, lr", buf);
    return COMMAND_NONE;
}


COMMAND_T update_console(State& state) {
    static char cmd_buf[32];
    static uint32_t cmd_buf_n = 0;

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
            cmd = fire_command(cmd_buf, cmd_buf_n, state);
            cmd_buf_n = 0;
            break;
        }
        c = uart_maybec(CONSOLE);
    }

    return cmd;
}