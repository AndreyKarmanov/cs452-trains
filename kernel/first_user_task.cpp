#include "first_user_task.h"
#include "clock_server.h"
#include "io_helpers.h"
#include "kernel_state.h"
#include "name_server.h"
#include "pathfind.h"
#include "rx_server.h"
#include "shell.h"
#include "syscall.h"
#include "sysinfo_task.h"
#include "tx_server.h"

#if (defined(PERF_TEST) && PERF_TEST) || (defined(RPS_TEST) && RPS_TEST) ||    \
    (defined(CLOCK_TEST) && CLOCK_TEST)
#include "message.h"
#include "rps_client.h"
#include "rps_server.h"
#include "test.h"
#endif

void first_user_task() {
  create(2, name_server_task);
  create(2, clock_server_task);
  create(2, tx_server_task);
  create(2, rx_server_task);

  auto tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  Puts(tx_tid, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__
               " / Andrey Karmanov / Anthony Ho\n\r");

#if defined(RPS_TEST) && RPS_TEST
  int rps_tid = create(1, rps_server_task);
  Printf(tx_tid, "RPS Server %d\n\r", rps_tid);
  int rps_task_tid = create(2, test_rps_task);
  Printf(tx_tid, "RPS Test Client %d\n\r", rps_tid);
  await_task(rps_task_tid);
#endif

#if defined(PERF_TEST) && PERF_TEST
  int timer_tid = create(1, test_timer_task);
  Printf(tx_tid, "Timer Test %d\n\r", timer_tid);
#endif

#if defined(CLOCK_TEST) && CLOCK_TEST
  auto p3_tid = create(3, test_clock_client_task);
  auto p4_tid = create(4, test_clock_client_task);
  auto p5_tid = create(5, test_clock_client_task);
  auto p6_tid = create(6, test_clock_client_task);

  Printf(tx_tid, "Created p3: %d, p4: %d, p5: %d, p6: %d\n\r", p3_tid, p4_tid,
         p5_tid, p6_tid);

  int rcv_tid;
  MessageVar rcv_msg;
  auto initalized = 0;
  while (initalized < 4) {
    receive(&rcv_tid, rcv_msg);
    if (!std::holds_alternative<FUT::ClientParamRequest>(rcv_msg)) {
      _assert(false, "FUT received invalid message");
    }

    if (rcv_tid == p3_tid) {
      reply(rcv_tid,
            FUT::ClientInitMessage{.delay_ticks = 10, .delay_count = 20});
    } else if (rcv_tid == p4_tid) {
      reply(rcv_tid,
            FUT::ClientInitMessage{.delay_ticks = 23, .delay_count = 9});
    } else if (rcv_tid == p5_tid) {
      reply(rcv_tid,
            FUT::ClientInitMessage{.delay_ticks = 33, .delay_count = 6});
    } else if (rcv_tid == p6_tid) {
      reply(rcv_tid,
            FUT::ClientInitMessage{.delay_ticks = 71, .delay_count = 3});
    } else {
      continue;
    }
    initalized++;
  }
#endif

  // Idle task
  create(PRIORITY_LEVELS - 1, idle_task);

  // header with idle, time
  create(PRIORITY_LEVELS - 2, sysinfo_task);

  // Shell
  create(PRIORITY_LEVELS - 2, shell_task);
}
