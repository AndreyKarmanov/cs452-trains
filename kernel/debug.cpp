#include "debug.h"
#include "uart.h"

bool _assert(bool value, const char *msg) {
  if (value)
    return false;

  uart_puts(CONSOLE, "\n\rASSERTION FAILED: ");
  uart_puts(CONSOLE, msg);
  uart_puts(CONSOLE, "\n\r");

  return true;
}