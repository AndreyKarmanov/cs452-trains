#include "cache.h"
#include "first_user_task.h"
#include "internal_syscall.h"
#include "kernel_state.h"
#include "mcp2515.h"
#include "rpi.h"
#include "scheduler.h"
#include "uart.h"
#include <optional>

#ifndef DATA_CACHE
#define DATA_CACHE 1
#endif

#ifndef INSTRUCTION_CACHE
#define INSTRUCTION_CACHE 1
#endif

#ifndef __OPTIMIZE__
#define __OPTIMIZE__ 0
#endif

extern "C" void setup_mmu(); // in mmu.S

extern "C" int kmain() {
#if defined(MMU)
  setup_mmu();
#endif
  gpio_init();
  gpio_init_interrupt();
  mcp2515_init();
  uart_config_and_enable(CONSOLE);

  data_cache_set(DATA_CACHE);
  instruction_cache_set(INSTRUCTION_CACHE);

  using namespace Kernel;
  _create(0, first_user_task);
  for (;;) {
    auto tid = scheduler.get_task();
    if (!tid.has_value()) {
      break; // error
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
  for (char *it = reinterpret_cast<char *>(s); n > 0; --n)
    *it++ = c;
  return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void *memcpy(void *dest, const void *src, size_t n) {
  char *sit   = reinterpret_cast<char *>(const_cast<void *>(src));
  char *cdest = reinterpret_cast<char *>(dest);
  for (size_t i = 0; i < n; ++i)
    *cdest++ = *sit++;
  return dest;
}
#endif
