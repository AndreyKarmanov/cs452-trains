#include "shell_new.h"

#include <ctype.h>

#include "debug.h"
#include "name_server.h"
#include "rx_server.h"
#include "syscall.h"
#include "tx_server.h"

constexpr static size_t RX_BUFFER_SIZE = 512;

static void shell_puts(int tx_tid, const char *s) {
  while (*s) {
    Putc(tx_tid, static_cast<unsigned char>(*s++));
  }
}

void shell_new_task() {
  int rx_tid = WhoIs(RX_Server::RX_SERVER_NAME);
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  _assert(rx_tid >= 0, "SHELL: RX SERVER WHOIS FAILED");
  _assert(tx_tid >= 0, "SHELL: TX SERVER WHOIS FAILED");

  char buffer[RX_BUFFER_SIZE];
  int buffer_index = 0;

  while (true) {
    int c = Getc(rx_tid);
    _assert(c >= 0, "SHELL: GETC FAILED");

    if (isprint(c) && buffer_index < static_cast<int>(RX_BUFFER_SIZE - 1)) {
      buffer[buffer_index++] = static_cast<char>(c);
      Putc(tx_tid, static_cast<unsigned char>(c));
    } else if ((c == 0x08 || c == 0x7f) && buffer_index > 0) {
      --buffer_index;
      shell_puts(tx_tid, "\b \b");
    } else if (c == '\r') {
      buffer[buffer_index] = '\0';
      shell_puts(tx_tid, "\n\r> ");

      // fire_command(buffer, buffer_index);
      buffer_index = 0;
    }
  }
}
