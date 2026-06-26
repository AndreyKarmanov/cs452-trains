#pragma once

#include "message.h"
#include "static_string.h"

int Getc(int tid);
int Putc(int tid, unsigned char c);
int Puts(int tid, const char *str);
int Printf(int tid, const char *fmt, ...);
int Debug_Puts(int tid, const char *str);

template <size_t SIZE> int Puts(int tid, const StaticString<SIZE> &str) {
  return Puts(tid, str.c_str());
}

template <typename... Args> int Debug_Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Debug_Puts(tid, str.c_str());
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
  str.set("\033[s\033[", DEBUG_LINE + offset, ";1H\033[K", args..., "\033[u");
  return Puts(tid, str);
}

template <typename... Args> int Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Puts(tid, str);
}
