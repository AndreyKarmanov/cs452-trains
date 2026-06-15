#include <cstddef>
#include <cstdarg>
#include <cstring>
#include <ctype.h>

#include "debug.h"
#include "map.h"
#include "shell_new.h"
#include "util.h"
#include "syscall.h"
#include "test.h"
#include "name_server.h"
#include "rx_server.h"
#include "tx_server.h"

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
    base = 16;
    p += 2;
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

static void fire_command(char *buf, size_t blen, int tx_tid) {
  char *cmd = skip_ws(buf);

  if (blen == 0)
    return;
  if (*cmd == '\0')
    return;

  if (strncmp(cmd, "q", 1) == 0) {
    exit();
  } else if (strncmp(cmd, "p", 1) == 0) {
    int parent_tid = my_parent_tid();
    console_printf(tx_tid, "My parent tid is %d\n\r", parent_tid);
  } else if (strncmp(cmd, "m", 1) == 0) {
    int tid = my_tid();
    console_printf(tx_tid, "My tid is %d\n\r", tid);
  } else if (strncmp(cmd, "y", 1) == 0) {
    yield();
    console_puts(tx_tid, "Yielded\n\r");
  } else if (strncmp(cmd, "c", 1) == 0) {
    int tid = create(3, shell_new_task);
    console_printf(tx_tid, "Created new shell %u", tid);
  } else if (strncmp(cmd, "d", 1) == 0) {
    char *cursor = cmd + 1;
    size_t address;
    size_t count = 16;

    if (!parse_hex_size_t(&cursor, &address)) {
      console_puts(tx_tid, "Usage: d <hex address> [count]\n\r");
    } else {
      cursor = skip_ws(cursor);
      if (*cursor != '\0' && !parse_count_size_t(&cursor, &count)) {
        console_puts(tx_tid, "Usage: d <hex address> [count]\n\r");
      } else {
        dump_memory_region(address, count);
      }
    }
  } else if (strncmp(cmd, "w", 1) == 0) {
    char *cursor = cmd + 1;
    size_t address;
    size_t value;

    if (!parse_hex_size_t(&cursor, &address)) {
      console_puts(tx_tid, "Usage: w <hex address> <hex value>\n\r");
    } else {
      cursor = skip_ws(cursor);
      if (!parse_hex_size_t(&cursor, &value)) {
        console_puts(tx_tid, "Usage: w <hex address> <hex value>\n\r");
      } else {
        write_memory_word(address, value);
        console_puts(tx_tid, "Wrote ");
        console_printf(tx_tid, "0x%x", static_cast<unsigned int>(value));
        console_puts(tx_tid, " to ");
        console_printf(tx_tid, "0x%x\n\r", static_cast<unsigned int>(address));
      }
    }
  } else if (strncmp(cmd, "t k1", 4) == 0) {
    int tid = create(2, test_k1);
    console_printf(tx_tid, "Created tid %u", tid);
  } else if (strncmp(cmd, "t map", 5) == 0) {
    test_map();
  } else if (strncmp(cmd, "t heap", 6) == 0) {
    test_heap();
  }else if (strncmp(cmd, "t event", 7) == 0){
    create(0, test_await_event_task);
  } else if (strncmp(cmd, "t clock", 7) == 0){
    create(0, test_clock_server);
  } else {
    console_puts(tx_tid, "Unknown command. Available: q (quit), p (parent tid), "
                         "m (my tid), y (yield), c (create), d (dump memory), "
                         "w (write memory), t k1, t map, t heap\n\r");
  }
}

void shell_new_task() {
  int rx_tid = WhoIs(RX_Server::RX_SERVER_NAME);
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  _assert(rx_tid >= 0, "SHELL: RX SERVER WHOIS FAILED");
  _assert(tx_tid >= 0, "SHELL: TX SERVER WHOIS FAILED");

  char buf[BUFFER_SIZE];
  size_t buf_n = 0;
  console_puts(tx_tid, "COMMANDS: q (quit) p (parent tid) m (my tid) y (yield) c "
                        "(create) d <hex address> [count] w <hex address> <hex value>\n\r> ");
  while (1) {
    int rc = Getc(rx_tid);
    _assert(rc >= 0, "SHELL: GETC FAILED");
    char c = static_cast<char>(rc);
    if (isprint(c) && buf_n < BUFFER_SIZE - 1) {
      buf[buf_n++] = c;
      console_putc(tx_tid, c);
    } else if ((c == 0x08 || c == 0x7f) && buf_n > 0) { // backspace
      console_puts(tx_tid, "\b \b"); // move back, print space, move back again
      --buf_n;
    } else if (c == '\r') { // enter
      buf[buf_n] = '\0';
      console_puts(tx_tid, "\n\r");
      fire_command(buf, buf_n, tx_tid);
      console_puts(tx_tid, "> ");
      buf_n = 0;
    }
  }
}

void console_putc(int tx_tid, char c) {
  Putc(tx_tid, static_cast<unsigned char>(c));
}

void console_puts(int tx_tid, const char *buf) {
  while (*buf) {
    console_putc(tx_tid, *buf++);
  }
}

void console_printf(int tx_tid, const char *fmt, ...) {
  va_list va;
  char ch, buf[12];

  va_start(va, fmt);
  while ((ch = *(fmt++))) {
    if (ch != '%')
      console_putc(tx_tid, ch);
    else {
      ch = *(fmt++);
      switch (ch) {
      case 'u':
        ui2a(va_arg(va, unsigned int), 10, buf);
        console_puts(tx_tid, buf);
        break;
      case 'd':
        i2a(va_arg(va, int), buf);
        console_puts(tx_tid, buf);
        break;
      case 'x':
        ui2a(va_arg(va, unsigned int), 16, buf);
        console_puts(tx_tid, buf);
        break;
      case 's':
        console_puts(tx_tid, va_arg(va, char *));
        break;
      case 'c':
        console_putc(tx_tid, va_arg(va, int));
        break;
      case '%':
        console_putc(tx_tid, ch);
        break;
      case '\0':
        va_end(va);
        return;
      }
    }
  }
  va_end(va);
}
