
#include "first_user_task.h"
#include "clock_server.h"
#include "idle_manager.h"
#include "name_server.h"
#include "rx_server.h"
#include "syscall.h"
#include "tx_server.h"

#if (defined(PERF_TEST) && PERF_TEST) || (defined(RPS_TEST) && RPS_TEST)
#include "message.h"
#include "rps_server.h"
#include "test.h"
#endif

void first_user_task() {
  create(2, name_server_task);
  create(2, clock_server_task);
  create(2, tx_server_task);
  create(2, rx_server_task);

  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  Puts(tx_tid, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__
               " / Andrey Karmanov / Anthony Ho\n\r");

  Puts(tx_tid, "Created name server, clock server, tx server, rx server\n\r");

#if defined(RPS_TEST) && RPS_TEST
  create(2, rps_server_task);
  Puts(tx_tid, "Created RPS server\n");
  await_task(create(1, test_rps_task));
#endif

#if defined(PERF_TEST) && PERF_TEST
  create(3, test_timer_task);
  Puts(tx_tid, "Created timer task\n");
#endif

  // Idle task
  create(0, idle_task);

  // Shell
  // create(0, shell_task);
}
