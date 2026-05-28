#include "debug.h"
#include "uart.h"

bool assert(bool value, const char *msg) {
  if (value)
    return false;

  uart_puts(CONSOLE, msg);

  return true;
}