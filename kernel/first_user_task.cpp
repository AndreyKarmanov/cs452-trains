
#include "first_user_task.h"
#include "name_server.h"
#include "shell.h"
#include "syscall.h"
#include "uart.h"

#if (defined(PERF_TEST) && PERF_TEST) || (defined(RPS_TEST) && RPS_TEST)
#include "message.h"
#include "rps_server.h"
#include "test.h"
#endif

void first_user_task() {
  create(2, name_server_task);
  uart_printf(CONSOLE, "Created name server\n");

#if defined(RPS_TEST) && RPS_TEST
  create(2, rps_server_task);
  uart_printf(CONSOLE, "Created RPS server\n");
  await_task(create(0, test_rps_task));
#endif

#if defined(PERF_TEST) && PERF_TEST
  create(3, test_timer_task);
  uart_printf(CONSOLE, "Created timer task\n");
#endif

  // Shell
  await_task(create(0, shell_task));
}
