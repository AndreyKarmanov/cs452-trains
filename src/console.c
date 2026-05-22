#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <ctype.h>

#include "console.h"
#include "uart.h"

#define CONSOLE_ROW "3"

void print_cmd_line(char buf[], uint32_t n) {
    uart_printf(CONSOLE, "\033[" CONSOLE_ROW ";1H\033[K>");
    uart_putl(CONSOLE, buf, n);
}

void clear_console(void) {
    uart_puts(CONSOLE, "\033[" CONSOLE_ROW ";1H\033[K>");
}

inline static size_t bite_blank(const char* buf, size_t blen) {
    size_t i = 0;

    while (i < blen && isblank(buf[i]))
        ++i;

    return i;
}

inline static size_t bite_string(const char* buf, size_t blen) {
    size_t i = 0;

    while (i < blen && isalpha(buf[i]))
        ++i;

    return i;
}

inline static size_t bite_int(const char* buf, size_t blen) {
    size_t i = 0;

    while (i < blen && isdigit(buf[i]))
        ++i;

    return i;
}

inline static uint32_t s2l(const char* buf, size_t blen) {
    uint32_t out = 0;

    size_t i = 0;
    while (i < blen && isdigit(buf[i]))
    {
        out *= 10;
        out += buf[i] - '0';
        ++i;
    }

    return out;
}

// parse and fire command 
static COMMAND_T fire_command(const char* buf, size_t blen) {
    if (blen == 0) {
        return COMMAND_NONE;
    }

    size_t start = bite_blank(buf, blen);
    size_t end = bite_string(buf + start, blen - start);

    if (end == 1 && strncmp(buf + start, "q", end) == 0) {
        return COMMAND_QUIT;
    }

    if (end == 2) {
        if (strncmp(buf + start, "tr", end) == 0) {
            start += end;
            start += bite_blank(buf + start, blen - start);

            end = bite_int(buf + start, blen - start);

            uart_printf(CONSOLE, "\033[4;1H\033[K>1r %u, %u, |%s|", start, end, buf + start);
            if (end == 0)
                return COMMAND_NONE;

            uint32_t loco_id = s2l(buf + start, end);

            start += end;
            start += bite_blank(buf + start, blen - start);

            end = bite_int(buf + start, blen - start);

            uart_printf(CONSOLE, "\033[4;1H\033[K>2r %u, %u, |%s|", start, end, buf + start);
            if (end == 0)
                return COMMAND_NONE;
            uint32_t speed = s2l(buf + start, end);

            start += end;
            start += bite_blank(buf + start, blen - start);

            if (start != blen)
                return COMMAND_NONE;

            uart_printf(CONSOLE, "\033[4;1H\033[K>tr %u, %u add %u len %u", loco_id, speed, start + end, blen);
        }
    }

    return COMMAND_NONE;
}

static char cmd_buf[32];
static uint32_t cmd_buf_n = 0;

COMMAND_T update_console() {
    COMMAND_T cmd = COMMAND_NONE;

    char c = uart_maybec(CONSOLE);
    if (c) {
        uart_printf(CONSOLE, "\033[" CONSOLE_ROW ";%uH", 2 + cmd_buf_n);
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