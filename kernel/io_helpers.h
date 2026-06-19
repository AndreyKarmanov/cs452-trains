#pragma once

#include "static_string.h"

int Getc(int tid);
int Putc(int tid, unsigned char c);
int Puts(int tid, const char *str);
int Printf(int tid, const char *fmt, ...);

template <size_t SIZE> int Puts(int tid, const StaticString<SIZE> &str) {
  return Puts(tid, str.c_str());
}
