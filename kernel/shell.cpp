#include <cstddef>
#include <cstring>
#include <ctype.h>

#include "debug.h"
#include "map.h"
#include "shell.h"
#include "syscall.h"
#include "test.h"
#include "uart_non_blocking.h"
#include "util.h"

#define BUFFER_SIZE 32

static char *skip_ws(char *p) {
  while (*p && isspace(static_cast<unsigned char>(*p)))
    ++p;
  return p;
}

static bool parse_hex_size_t(char **cursor, size_t *value) {
  char *p = skip_ws(*cursor);

  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
    p += 2;

  size_t result = 0;
  bool found    = false;

  while (*p) {
    int digit = a2d(*p);
    if (digit < 0 || digit >= 16)
      break;
    result = (result << 4) | static_cast<size_t>(digit);
    found  = true;
    ++p;
  }

  if (!found || (*p && !isspace(static_cast<unsigned char>(*p))))
    return false;

  *cursor = p;
  *value  = result;
  return true;
}

static bool parse_count_size_t(char **cursor, size_t *value) {
  char *p = skip_ws(*cursor);

  if (*p == '\0')
    return false;

  size_t base = 10;
  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
    base  = 16;
    p    += 2;
  }

  size_t result = 0;
  bool found    = false;

  while (*p) {
    int digit = a2d(*p);
    if (digit < 0 || static_cast<size_t>(digit) >= base)
      break;
    result = result * base + static_cast<size_t>(digit);
    found  = true;
    ++p;
  }

  if (!found || (*p && !isspace(static_cast<unsigned char>(*p))))
    return false;

  *cursor = p;
  *value  = result;
  return true;
}

void fire_command(char *buf, size_t blen, UARTNB &uart) {
  char *cmd = skip_ws(buf);

  if (blen == 0)
    return;
  if (*cmd == '\0')
    return;

  if (strncmp(cmd, "q", 1) == 0) {
    exit();
  } else if (strncmp(cmd, "p", 1) == 0) {
    int parent_tid = my_parent_tid();
    uart.printf("My parent tid is %d\n\r", parent_tid);
  } else if (strncmp(cmd, "m", 1) == 0) {
    int tid = my_tid();
    uart.printf("My tid is %d\n\r", tid);
  } else if (strncmp(cmd, "y", 1) == 0) {
    yield();
    uart.puts("Yielded\n\r");
  } else if (strncmp(cmd, "c", 1) == 0) {
    int tid = create(3, shell_task);
    uart.printf("Created new shell %u", tid);
  } else if (strncmp(cmd, "d", 1) == 0) {
    char *cursor = cmd + 1;
    size_t address;
    size_t count = 16;

    if (!parse_hex_size_t(&cursor, &address)) {
      uart.puts("Usage: d <hex address> [count]\n\r");
    } else {
      cursor = skip_ws(cursor);
      if (*cursor != '\0' && !parse_count_size_t(&cursor, &count)) {
        uart.puts("Usage: d <hex address> [count]\n\r");
      } else {
        dump_memory_region(address, count);
      }
    }
  } else if (strncmp(cmd, "w", 1) == 0) {
    char *cursor = cmd + 1;
    size_t address;
    size_t value;

    if (!parse_hex_size_t(&cursor, &address)) {
      uart.puts("Usage: w <hex address> <hex value>\n\r");
    } else {
      cursor = skip_ws(cursor);
      if (!parse_hex_size_t(&cursor, &value)) {
        uart.puts("Usage: w <hex address> <hex value>\n\r");
      } else {
        write_memory_word(address, value);
        uart.puts("Wrote ");
        uart.printf("0x%x", static_cast<unsigned int>(value));
        uart.puts(" to ");
        uart.printf("0x%x\n\r", static_cast<unsigned int>(address));
      }
    }
  } else if (strncmp(cmd, "t k1", 4) == 0) {
    int tid = create(2, test_k1);
    uart.printf("Created tid %u", tid);
  } else if (strncmp(cmd, "t map", 5) == 0) {
    test_map();
  } else if (strncmp(cmd, "t heap", 6) == 0) {
    test_heap();
  } else if (strncmp(cmd, "t event", 7) == 0) {
    create(0, test_await_event_task);
  } else if (strncmp(cmd, "t clock", 7) == 0) {
    create(0, test_clock_server);
  } else {
    uart.puts("Unknown command. Available: q (quit), p (parent tid), "
              "m (my tid), y (yield), c (create), d (dump memory), "
              "w (write memory), t k1, t map, t heap\n\r");
  }
}

void shell_task() {
  char buf[BUFFER_SIZE];
  size_t buf_n = 0;
  UARTNB uart(CONSOLE);
  uart.puts(
      "COMMANDS: q (quit) p (parent tid) m (my tid) y (yield) c "
      "(create) d <hex address> [count] w <hex address> <hex value>\n\r> ");
  while (1) {
    if (uart.can_receive_io()) {
      char c = uart.getc();
      if (isprint(c) && buf_n < BUFFER_SIZE - 1) {
        buf[buf_n++] = c;
        uart.putc(c);
      } else if ((c == 0x08 || c == 0x7f) && buf_n > 0) { // backspace
        uart.puts("\b \b"); // move back, print space, move back again
        --buf_n;
      } else if (c == '\r') { // enter
        buf[buf_n] = '\0';
        uart.puts("\n\r");
        fire_command(buf, buf_n, uart);
        uart.puts("> ");
        buf_n = 0;
      }
    } else {
      yield();
    }
    uart.send_io();
  }
}