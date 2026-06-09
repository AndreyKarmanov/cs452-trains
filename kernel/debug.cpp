#include "debug.h"
#include <cstdint>

#include "uart.h"

namespace {
void put_hex(size_t value, size_t width) {
  static const char hex_digits[] = "0123456789abcdef";

  uart_puts(CONSOLE, "0x");
  for (size_t shift = width; shift > 0; --shift) {
    size_t digit = (value >> ((shift - 1) * 4)) & 0x0f;
    uart_putc(CONSOLE, hex_digits[digit]);
  }
}
} // namespace

bool _assert(bool value, const char *msg) {
  if (value)
    return false;

  uart_puts(CONSOLE, "\n\rASSERTION FAILED: ");
  uart_puts(CONSOLE, msg);
  uart_puts(CONSOLE, "\n\r");

  return true;
}

void dump_memory_region(size_t address, size_t count) {
  auto *words = reinterpret_cast<volatile const uint32_t *>(address);

  uart_puts(CONSOLE, "\n\rAddress             ");
  uart_puts(CONSOLE, "+00        +04        +08        +0c\n\r");

  for (size_t i = 0; i < count; i += 4) {
    put_hex(address + i * sizeof(uint32_t), sizeof(size_t) * 2);
    uart_puts(CONSOLE, ": ");

    for (size_t j = 0; j < 4; ++j) {
      if (i + j < count) {
        put_hex(words[i + j], 8);
      } else {
        uart_puts(CONSOLE, "          ");
      }
      uart_putc(CONSOLE, ' ');
    }

    uart_puts(CONSOLE, "\n\r");
  }
}

void write_memory_word(size_t address, size_t value) {
  *reinterpret_cast<volatile uint32_t *>(address) = static_cast<uint32_t>(value);
}