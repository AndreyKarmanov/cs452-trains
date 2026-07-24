#pragma once

#include "message.h"
#include "static_string.h"
#include "syscall.h"
#include <cstring>

int Getc(int tid);

template <size_t SIZE> int Puts(int tid, const StaticString<SIZE> &str) {
  size_t offset    = 0;
  size_t total_len = str.len;

  while (offset < total_len) {
    size_t chunk = total_len - offset;
    if (chunk > static_cast<size_t>(TX::MAX_DATA_LENGTH)) {
      chunk = TX::MAX_DATA_LENGTH;
    }

    TX::SendMsg send_msg{};
    send_msg.len = static_cast<int>(chunk);
    std::memcpy(send_msg.data, str.data + offset, chunk);
    auto rcv_msg = send<TX::ReplyMsg>(tid, send_msg);
    if (!rcv_msg.has_value()) {
      return -1;
    }

    offset += chunk;
  }

  return 0;
}

template <size_t SIZE> int Debug_Puts(int tid, const StaticString<SIZE> &str) {
  static constexpr int DEBUG_LINE_START = 40;
  static int debug_scroll_line          = 0;

  const int row = DEBUG_LINE_START + debug_scroll_line;
#if !defined(DATA_COLLECTION) || !DATA_COLLECTION
  debug_scroll_line = (debug_scroll_line + 1) % 40;
#else
  debug_scroll_line = (debug_scroll_line + 1);
#endif
  StaticString<TX::MAX_DATA_LENGTH> out;
  out.set("\033[s\033[", row, ";1H\033[K", str, "\n\r\033[K\033[u");
  return Puts(tid, out);
}

template <typename... Args> int Debug_Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Debug_Puts(tid, str);
}

template <size_t SIZE, typename T>
bool AppendPadded(StaticString<SIZE> &str, const T &value, size_t width,
                  char fill = ' ') {
  StaticString<SIZE> value_str;
  value_str.append(value);

  if (value_str.len >= width) {
    return str.append(value_str);
  }

  for (size_t i = value_str.len; i < width; ++i) {
    if (!str.append(fill)) {
      return false;
    }
  }

  return str.append(value_str);
}

template <typename... Args>
int Offset_Puts(int tid, int offset, const Args &...args) {
  static constexpr int DEBUG_LINE = 40;
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set("\033[s\033[", DEBUG_LINE + offset, ";1H", args..., "\033[u");
  return Puts(tid, str);
}

template <typename... Args> int Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Puts(tid, str);
}
