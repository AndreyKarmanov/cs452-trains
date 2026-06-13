
#include "first_user_task.h"
#include "clock_server.h"
#include "debug.h"
#include "idle_manager.h"
#include "kernel_state.h"
#include "message.h"
#include "name_server.h"
#include "shell.h"
#include "syscall.h"
#include "test.h"
#include "uart.h"

#if (defined(PERF_TEST) && PERF_TEST) || (defined(RPS_TEST) && RPS_TEST) ||    \
    (defined(CLOCK_TEST) && CLOCK_TEST)
#include "message.h"
#include "rps_server.h"
#include "test.h"
#endif

void first_user_task() {
  uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__
                     " / Andrey Karmanov / Anthony Ho\n\r");

  int ns_tid = create(0, name_server_task);
  uart_printf(CONSOLE, "Name Server %d\n\r", ns_tid);

  int cs_tid = create(0, clock_server_task);
  uart_printf(CONSOLE, "Clock Server %d\n\r", cs_tid);

#if defined(RPS_TEST) && RPS_TEST
  int rps_tid = create(1, rps_server_task);
  uart_printf(CONSOLE, "RPS Server %d\n\r", rps_tid);
  int rps_task_tid = create(2, test_rps_task);
  uart_printf(CONSOLE, "RPS Test Client %d\n\r", rps_tid);
  await_task(rps_task_tid);
#endif

#if defined(PERF_TEST) && PERF_TEST
  int timer_tid = create(1, test_timer_task);
  uart_printf(CONSOLE, "Timer Test %d\n\r", timer_tid);
#endif

#if defined(CLOCK_TEST) && CLOCK_TEST
  auto p3_tid = create(3, test_clock_client_task);
  auto p4_tid = create(4, test_clock_client_task);
  auto p5_tid = create(5, test_clock_client_task);
  auto p6_tid = create(6, test_clock_client_task);

  uart_printf(CONSOLE, "Created p3: %d, p4: %d, p5: %d, p6: %d\n\r", p3_tid,
              p4_tid, p5_tid, p6_tid);

  int rcv_tid;
  Message rcv_msg;
  Message reply_msg;
  reply_msg.type  = MessageType::FUT_CLIENT_PARAMS_REPLY;
  auto initalized = 0;
  while (initalized < 4) {
    int rcv_len = receive(&rcv_tid, rcv_msg);
    _assert(rcv_msg.type == MessageType::FUT_CLIENT_PARAMS &&
                rcv_len == static_cast<int>(sizeof(rcv_msg)),
            "FUT received invalid message");
    if (rcv_tid == p3_tid) {
      reply_msg.data.fut_params.delay_ticks = 10;
      reply_msg.data.fut_params.delay_count = 20;
    } else if (rcv_tid == p4_tid) {
      reply_msg.data.fut_params.delay_ticks = 23;
      reply_msg.data.fut_params.delay_count = 9;
    } else if (rcv_tid == p5_tid) {
      reply_msg.data.fut_params.delay_ticks = 33;
      reply_msg.data.fut_params.delay_count = 6;
    } else if (rcv_tid == p6_tid) {
      reply_msg.data.fut_params.delay_ticks = 71;
      reply_msg.data.fut_params.delay_count = 3;
    } else {
      continue;
    }
    initalized++;
    reply(rcv_tid, reply_msg);
  }
#endif

  // Idle task
  create(PRIORITY_LEVELS - 1, idle_task);

  // Shell
  // create(0, shell_task);
}
