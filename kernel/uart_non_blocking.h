#ifndef _uart_nb_h_
#define _uart_nb_h_ 1

#include "buffer.h"
#include <cstddef>

#define CONSOLE 0

class UARTNB {
  static constexpr size_t TX_BUFFER_SIZE = 1024;
  size_t line;

  Buffer<char, TX_BUFFER_SIZE> tx_buffer;

public:
  bool can_receive_io();
  char getc();
  void send_io();
  UARTNB(size_t line) : line(line) {}

  void putc(char c);
  void putl(const char *buf, size_t blen);
  void puts(const char *buf);
  void printf(const char *fmt, ...);
};

#endif /* uart_non_blocking.h */
