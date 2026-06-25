#include "io_helpers.h"
#include "message.h"
#include "static_string.h"
#include "syscall.h"
#include "util.h"
#include <cstdarg>
#include <cstring>

static constexpr int DEBUG_LINE_START = 40;
static int debug_scroll_line          = 0;

int Debug_Puts(int tid, const char *str) {
  const int row     = DEBUG_LINE_START + debug_scroll_line;
  debug_scroll_line = (debug_scroll_line + 1) % 40;

  StaticString<TX::MAX_DATA_LENGTH> out;
  out.set("\033[s\033[", row, ";1H\033[K", str, "\033[u");
  return Puts(tid, out.c_str());
}

// tid should be the RX server tid
int Getc(int tid) {
  auto rcv_msg = send<RX::GetcReplyMsg>(tid, RX::GetcMsg{});
  if (!rcv_msg.has_value()) {
    return -1;
  }
  return static_cast<int>(rcv_msg->c);
}

// tid should be the TX server tid
int Putc(int tid, unsigned char c) {
  TX::SendMsg msg{};
  msg.len      = 1;
  msg.data[0]  = static_cast<char>(c);
  auto rcv_msg = send<TX::ReplyMsg>(tid, msg);
  return rcv_msg.has_value() ? 0 : -1;
}

// tid should be the TX server tid. Batched version of Putc.
int Puts(int tid, const char *str) {
  size_t offset    = 0;
  size_t total_len = strlen(str);

  while (offset < total_len) {
    size_t chunk = total_len - offset;
    if (chunk > static_cast<size_t>(TX::MAX_DATA_LENGTH)) {
      chunk = TX::MAX_DATA_LENGTH;
    }

    TX::SendMsg send_msg{};
    send_msg.len = static_cast<int>(chunk);
    std::memcpy(send_msg.data, str + offset, chunk);
    auto rcv_msg = send<TX::ReplyMsg>(tid, send_msg);
    if (!rcv_msg.has_value()) {
      return -1;
    }

    offset += chunk;
  }

  return 0;
}

static void printf_flush(int tid, char *buffer, size_t &buffer_index) {
  buffer[buffer_index] = '\0';
  Puts(tid, buffer);
  buffer_index = 0;
}

// tid should be the TX server tid
// assumes that string format items won't exceed MAX_DATA_LENGTH - 1
int Printf(int tid, const char *fmt, ...) {
  char buffer[TX::MAX_DATA_LENGTH];
  size_t buffer_index = 0;
  va_list va;

  char ch, temp_buffer[32];
  const char *str;
  va_start(va, fmt);

  while ((ch = *(fmt++))) {
    if (ch != '%') {
      if (buffer_index >= TX::MAX_DATA_LENGTH - 1) {
        printf_flush(tid, buffer, buffer_index);
      }
      buffer[buffer_index++] = ch;
    } else {
      ch  = *(fmt++);
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
        str            = temp_buffer;
        break;
      case 'b':
        ui2a(va_arg(va, unsigned int), 2, temp_buffer);
        str = temp_buffer;
        break;
      case '%':
        temp_buffer[0] = '%';
        temp_buffer[1] = '\0';
        str            = temp_buffer;
        break;
      case '\0':
        break;
      }

      if (str) {
        size_t str_len = strlen(str);
        if (buffer_index + str_len >= TX::MAX_DATA_LENGTH - 1) {
          printf_flush(tid, buffer, buffer_index);
        }
        std::memcpy(buffer + buffer_index, str, str_len);
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
