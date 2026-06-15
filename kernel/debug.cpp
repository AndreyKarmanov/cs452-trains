#include "debug.h"
#include "uart.h"
#include <cstdint>
#include <source_location>

namespace {
  void put_hex(size_t value, size_t width) {
    static const char hex_digits[] = "0123456789abcdef";

    debug_puts(CONSOLE, "0x");
    for (size_t shift = width; shift > 0; --shift) {
      size_t digit = (value >> ((shift - 1) * 4)) & 0x0f;
      debug_putc(CONSOLE, hex_digits[digit]);
    }
  }
} // namespace

bool _assert(bool condition, const char *msg, const std::source_location loc) {
  if (!condition) {
    debug_printf(CONSOLE, "FAIL: %s at %s:%u\n", msg, loc.file_name(),
                 loc.line());
  }
  return condition;
}

void dump_memory_region(size_t address, size_t count) {
  auto *words = reinterpret_cast<volatile const uint32_t *>(address);

  debug_puts(CONSOLE, "\n\rAddress             ");
  debug_puts(CONSOLE, "+00        +04        +08        +0c\n\r");

  for (size_t i = 0; i < count; i += 4) {
    put_hex(address + i * sizeof(uint32_t), sizeof(size_t) * 2);
    debug_puts(CONSOLE, ": ");

    for (size_t j = 0; j < 4; ++j) {
      if (i + j < count) {
        put_hex(words[i + j], 8);
      } else {
        debug_puts(CONSOLE, "          ");
      }
      debug_putc(CONSOLE, ' ');
    }

    debug_puts(CONSOLE, "\n\r");
  }
}

void write_memory_word(size_t address, size_t value) {
  *reinterpret_cast<volatile uint32_t *>(address) =
      static_cast<uint32_t>(value);
}