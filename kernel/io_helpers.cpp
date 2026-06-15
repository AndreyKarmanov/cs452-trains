#include "io_helpers.h"
#include "message.h"
#include "syscall.h"
#include <cstdarg>
#include <cstring>

#include "util.h"

// tid should be the RX server tid
int Getc(int tid) {
  Message msg;
  msg.type = MessageType::RX_GETC;
  Message rcv_msg;
  auto rcv_len = send(tid, msg, rcv_msg);

  if (rcv_len < static_cast<int>(sizeof(rcv_msg)) ||
      rcv_msg.type != MessageType::RX_GETC_REPLY) {
    return -1;
  }
  return static_cast<int>(rcv_msg.data.rx_getc_reply.c);
}

// tid should be the TX server tid
int Putc(int tid, unsigned char c) {
  Message msg;
  msg.type                 = MessageType::TX_SEND;
  msg.data.tx_send.len     = 1;
  msg.data.tx_send.data[0] = c;
  Message rcv_msg;
  auto rcv_len = send(tid, msg, rcv_msg);

  return rcv_len < 0 ? -1 : 0;
}

// tid should be the TX server tid. Batched version of Putc.
int Puts(int tid, const char *str) {
  Message msg;
  msg.type             = MessageType::TX_SEND;
  msg.data.tx_send.len = strlen(str);
  __builtin_memcpy(msg.data.tx_send.data, str, msg.data.tx_send.len);
  Message rcv_msg;
  auto rcv_len = send(tid, msg, rcv_msg);

  return rcv_len < 0 ? -1 : 0;
}

// tid should be the TX server tid
// warning: does not check for buffer overflow
int Printf(int tid, const char *fmt, ...) {
  char buffer[TX::MAX_DATA_LENGTH];
  size_t buffer_index = 0;
  va_list va;

  char ch, temp_buffer[32];
  const char *str;
  va_start(va, fmt);

  while ((ch = *(fmt++))) {
    if (ch != '%') {
      buffer[buffer_index++] = ch;
    } else {
      ch = *(fmt++);
      str = nullptr;
      switch (ch) {
      case 'u':
        ui2a(va_arg(va, unsigned int), 10, temp_buffer);
        str = temp_buffer;
        break;
      case 'd':
        i2a(va_arg(va, int), temp_buffer);
        str = temp_buffer;
        break;
      case 'x':
        ui2a(va_arg(va, unsigned int), 16, temp_buffer);
        str = temp_buffer;
        break;
      case 's':
        str = va_arg(va, const char *);
        break;
      case 'c':
        temp_buffer[0] = static_cast<char>(va_arg(va, int));
        temp_buffer[1] = '\0';
        str = temp_buffer;
        break;
      case '%':
        temp_buffer[0] = '%';
        temp_buffer[1] = '\0';
        str = temp_buffer;
        break;
      case '\0':
        break;
      }

      if (str) {
        size_t str_len = strlen(str);
        __builtin_memcpy(buffer + buffer_index, str, str_len);
        buffer_index += str_len;
      }

      if (ch == '\0') {
        break;
      }
    }
  }

  va_end(va);
  buffer[buffer_index] = '\0';
  return Puts(tid, buffer);
}
