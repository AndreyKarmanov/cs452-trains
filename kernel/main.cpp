#include <optional>

#include "cache.h"
#include "first_user_task.h"
#include "internal_syscall.h"
#include "kernel_state.h"
#include "multi_core.h"
#include "rpi.h"
#include "scheduler.h"
#include "shell.h"
#include "uart.h"

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

void test_task() {
  while (1) {
    uart_puts(CONSOLE, "Hello from test task!\n\r");
  }
}

extern "C" int kmain() {
#if defined(MMU)
  setup_mmu();
#endif
  gpio_init();
  uart_config_and_enable(CONSOLE);

  data_cache_set(DATA_CACHE);
  instruction_cache_set(INSTRUCTION_CACHE);
  uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__
                     " / Andrey Karmanov / Anthony Ho\n\r");
  uart_printf(CONSOLE,
              "Kernel initialized DATA_CACHE: %u INSTRUCTION_CACHE: %u "
              "OPTIMIZATION: %u\n\r",
              DATA_CACHE, INSTRUCTION_CACHE, __OPTIMIZE__);

  using namespace Kernel;

  // After dropping to EL1, in core 0:

  auto td = require_td(_create(1, test_task));
  launch_pinned_task(1, td); // launch shell task on core 1

  // uart_getc(CONSOLE); // wait for a key press to start the shell

  int tid = _create(1, first_user_task);
  scheduler.schedule(require_td(tid));

  for (;;) {
    auto tid = scheduler.get_task();
    if (!tid.has_value()) {
      if (task_allocator.allocated_count() == 1) { // only name server task left
        uart_puts(CONSOLE, "No tasks left, halting.\n\r");
        break;
      }
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
