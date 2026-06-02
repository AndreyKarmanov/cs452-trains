#include <optional>

#include "first_user_task.h"
#include "internal_syscall.h"
#include "kernel_state.h"
#include "rpi.h"
#include "scheduler.h"
#include "uart.h"

extern "C" void setup_mmu(); // in mmu.S

extern "C" int kmain() {
#if defined(MMU)
  setup_mmu();
#endif
  gpio_init();
  uart_config_and_enable(CONSOLE);

  uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__
                     " / Andrey Karmanov / Anthony Ho\n\r");

  using namespace Kernel;
  _create(1, first_user_task);

  for (;;) {
    auto tid = scheduler.get_task();
    if (!tid.has_value()) {
      continue; // no ready tasks, spin
    }
    auto active_tid = tid.value();
    auto request    = activate(active_tid);
    handle(active_tid, request);
  }

  return 0;
}

#if !defined(MMU)
#include <cstddef>

// define our own memset to avoid SIMD instructions emitted from the compiler
void *memset(void *s, int c, size_t n) {
  for (char *it = (char *)s; n > 0; --n)
    *it++ = c;
  return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void *memcpy(void *dest, const void *src, size_t n) {
  char *sit   = (char *)src;
  char *cdest = (char *)dest;
  for (size_t i = 0; i < n; ++i)
    *cdest++ = *sit++;
  return dest;
}
#endif
