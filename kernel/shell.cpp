#include <ctype.h>

#include "clock_server.h"
#include "debug.h"
#include "heap.h"
#include "io_helpers.h"
#include "kernel_state.h"
#include "map.h"
#include "mcp2515.h"
#include "name_server.h"
#include "shell.h"
#include "syscall.h"
#include "test.h"
#include "train_ui_server.h"
#include "uart_rx_server.h"
#include "uart_tx_server.h"
#include "util.h"
#include <cstddef>
#include <cstring>
#include <ctype.h>

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
    Printf(tx_tid, "My parent tid is %d\n\r", parent_tid);
  } else if (strncmp(cmd, "m", 1) == 0) {
    int tid = my_tid();
    Printf(tx_tid, "My tid is %d\n\r", tid);
  } else if (strncmp(cmd, "y", 1) == 0) {
    yield();
    Puts(tx_tid, "Yielded\n\r");
  } else if (strncmp(cmd, "c", 1) == 0) {
    int tid = create(3, shell_task);
    Printf(tx_tid, "Created new shell %u", tid);
  } else if (strncmp(cmd, "d", 1) == 0) {
    char *cursor = cmd + 1;
    size_t address;
    size_t count = 16;

    if (!parse_hex_size_t(&cursor, &address)) {
      Puts(tx_tid, "Usage: d <hex address> [count]\n\r");
    } else {
      cursor = skip_ws(cursor);
      if (*cursor != '\0' && !parse_count_size_t(&cursor, &count)) {
        Puts(tx_tid, "Usage: d <hex address> [count]\n\r");
      } else {
        dump_memory_region(address, count);
      }
    }
  } else if (strncmp(cmd, "w", 1) == 0) {
    char *cursor = cmd + 1;
    size_t address;
    size_t value;

    if (!parse_hex_size_t(&cursor, &address)) {
      Puts(tx_tid, "Usage: w <hex address> <hex value>\n\r");
    } else {
      cursor = skip_ws(cursor);
      if (!parse_hex_size_t(&cursor, &value)) {
        Puts(tx_tid, "Usage: w <hex address> <hex value>\n\r");
      } else {
        write_memory_word(address, value);
        Puts(tx_tid, "Wrote ");
        Printf(tx_tid, "0x%x", static_cast<unsigned int>(value));
        Puts(tx_tid, " to ");
        Printf(tx_tid, "0x%x\n\r", static_cast<unsigned int>(address));
      }
    }
  } else if (strncmp(cmd, "t k1", 4) == 0) {
    int tid = create(2, test_k1);
    Printf(tx_tid, "Created tid %u", tid);
  } else if (strncmp(cmd, "t map", 5) == 0) {
    test_map();
  } else if (strncmp(cmd, "t heap", 6) == 0) {
    test_heap();
  } else if (strncmp(cmd, "t event", 7) == 0) {
    create(0, test_await_event_task);
  } else if (strncmp(cmd, "t clock", 7) == 0) {
    create(0, test_clock_server);
  } else if (strncmp(cmd, "t name_server", 13) == 0) {
    create(0, test_name_server);
  } else if (strncmp(cmd, "t cycles", 8) == 0) {
    Printf(tx_tid, "Syscall cycle counts:\n\r");
    for (const auto &[k, v] : Kernel::syscall_cycle_counts) {
      auto total_cycles = Kernel::syscall_cycle_totals.get(k).value_or(1);
      Printf(tx_tid, "  %d: %d cycles\n\r", k, v / total_cycles);
    }
  } else if (strncmp(cmd, "t ssr", 5) == 0) {
    create(1, test_timer_task);
  } else if (strncmp(cmd, "t canf", 6) == 0) {
    Printf(tx_tid, "CAN irq flags: %b\n\r", mcp2515_get_active_irq());
  } else if (strncmp(cmd, "t cane", 6) == 0) {
    Printf(tx_tid, "CAN enabled irq: %b\n\r", mcp2515_get_enabled_interrupt());
  } else if (strncmp(cmd, "t cani", 6) == 0) {
    Printf(tx_tid, "CAN irq source: %b\n\r", mcp2515_get_irq_source());
  } else if (strncmp(cmd, "t cans", 6) == 0) {
    create(1, test_can_tx_irq_task);
  } else if (strncmp(cmd, "t can", 5) == 0) {
    create(1, test_can_rx_irq_task);
  } else if (strncmp(cmd, "train", 5) == 0) {
    create(1, train_controller_program_task);
    await_event(Event::NEVER);
  } else if (strncmp(cmd, "tree", 4) == 0) {
    // test_tree();
  } else {
    Puts(tx_tid, "Unknown: p (parent tid), "
                 "m (my tid), y (yield), c (create), d (dump memory), "
                 "w (write memory), t k1, t map, t heap\n\r");
  }
}

void shell_task() {
  int rx_tid = WhoIs(UART_RX_Server::RX_SERVER_NAME);
  int tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(rx_tid >= 0, "SHELL: RX SERVER WHOIS FAILED");
  _assert(tx_tid >= 0, "SHELL: TX SERVER WHOIS FAILED");
  Puts(tx_tid, "\033[2J\033[?25l\033[2;1H" __DATE__ " / " __TIME__
               " / Andrey Karmanov / Anthony Ho\n\r");

  char buf[BUFFER_SIZE];
  size_t buf_n = 0;
  Puts(tx_tid, "COMMANDS: trains (run trains) | d(ump) <hex address> [count] | "
               "w(write) <hex address> <hex value>\n\r> ");
  while (1) {
    int rc = Getc(rx_tid);
    _assert(rc >= 0, "SHELL: GETC FAILED");
    char c = static_cast<char>(rc);
    if (isprint(c) && buf_n < BUFFER_SIZE - 1) {
      buf[buf_n++] = c;
      Putc(tx_tid, c);
    } else if ((c == 0x08 || c == 0x7f) && buf_n > 0) { // backspace
      Puts(tx_tid, "\b \b"); // move back, print space, move back again
      --buf_n;
    } else if (c == '\r') { // enter
      buf[buf_n] = '\0';
      Puts(tx_tid, "\n\r");
      fire_command(buf, buf_n, tx_tid);
      Puts(tx_tid, "> ");
      buf_n = 0;
    }
  }
}
