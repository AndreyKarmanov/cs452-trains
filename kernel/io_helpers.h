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
  str.append(args...);
  return Debug_Puts(tid, str.c_str());
}

template <typename... Args>
int Offset_Puts(int tid, int offset, const Args &...args) {
  static constexpr int DEBUG_LINE = 40;
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.append("\033[s\033[", DEBUG_LINE + offset, ";1H", args..., "\033[u");
  return Puts(tid, str);
}

template <typename... Args> int Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Puts(tid, str);
}
