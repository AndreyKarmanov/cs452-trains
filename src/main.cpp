#include "cache.h"
#include "first_user_task.h"
#include "gic.h"
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
  gic_init();
  mcp2515_init();
  uart_config_and_enable(CONSOLE);
  uart_config_and_enable(WEBSERIAL);
  data_cache_set(DATA_CACHE);
  instruction_cache_set(INSTRUCTION_CACHE);

  auto tid = kernel_runtime.task_table.create_task(0, first_user_task, 0);
  kernel_runtime.scheduler.schedule(tid, 0);
  for (;;) {
    auto tid = kernel_runtime.scheduler.get_task();
    if (!tid.has_value()) {
      _assert(false, "no tasks to schedule");
      break; // error
    }
    auto active_tid = tid.value();
    auto td         = kernel_runtime.task_table.lookup_td(active_tid);
    if (td == nullptr) {
      _assert(false, "invalid tid");
      break;
    }
    auto request = activate_task(*td);
    handle(active_tid, request);
  }
  debug_printf(CONSOLE, "KERNEL HALTED\n\r");
  return 0;
}