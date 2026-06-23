#include "sysinfo_task.h"
#include "clock_server.h"
#include "io_helpers.h"
#include "name_server.h"
#include "syscall.h"
#include "time.h"
#include "uart_tx_server.h"

void sysinfo_task() {
  static constexpr uint32_t UPDATE_INTERVAL_TICKS = TICKS_PER_S / 10;
  static constexpr int SYSINFO_ROW                = 1;
  auto cs_tid                                     = WhoIs(ClockServer<>::NAME);
  auto tx_tid                                     = WhoIs(UART_TX_Server::NAME);
  auto ticks                                      = Time(cs_tid);

  while (true) {
    auto time_us       = static_cast<uint64_t>(ticks) * TICK_TIME_US;
    auto idle_pct      = kernel_idle_pct();
    auto total_seconds = time_us / TIME_1S_US;
    auto minutes       = (total_seconds / 60) % 60;
    auto seconds       = total_seconds % 60;
    auto tenths        = (time_us / 100'000) % 10;

    Puts(tx_tid, "\033[s\033[", SYSINFO_ROW, ";1H", "TIME: ", (minutes / 10),
         (minutes % 10), ":", (seconds / 10), (seconds % 10), ".", tenths,
         " IDLE: ", idle_pct, "%\033[u");

    ticks = DelayUntil(cs_tid, ticks + UPDATE_INTERVAL_TICKS);
  }
}