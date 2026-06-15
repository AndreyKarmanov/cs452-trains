#include "clock_server.h"
#include "debug.h"
#include "message.h"
#include "syscall.h"
#include "uart.h"

static void clock_notifier_task() {
  int cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  uart_printf(CONSOLE, "STARTED CLOCK NOTIFIER\n\r");

  while (true) {
    await_event(Event::CLOCK_TICK_1MS);
    [[maybe_unused]] auto rcv_msg = sendVariant<CS::Tick>(cs_tid, CS::Tick{});
  }
}

void clock_server_task() {
  ClockServer<> clock_server;
  create(2, clock_notifier_task);
  while (true) {
    clock_server.run();
    yield();
  }
}

int Time(int tid) {
  auto res = sendVariant<CS::TimeReply>(tid, CS::Time{});
  if (!res) {
    return res.error();
  }
  return res->ticks;
}

int Delay(int tid, uint32_t ticks) {
  auto res = sendVariant<CS::DelayReply>(tid, CS::Delay{.ticks = ticks});
  if (!res) {
    return res.error();
  }
  return res->ticks;
}

int DelayUntil(int tid, uint32_t ticks) {
  auto res = sendVariant<CS::DelayReply>(tid, CS::DelayUntil{.ticks = ticks});
  if (!res) {
    return res.error();
  }
  return res->ticks;
}

void test_clock_server() {
  auto cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);

  auto time = Time(cs_tid);
  uart_printf(CONSOLE, "Current time: %d ticks\n\r", time);

  time = Delay(cs_tid, 500);
  uart_puts(CONSOLE, "5 second delay\n\r");

  time = DelayUntil(cs_tid, time + 500);
  uart_puts(CONSOLE, "5 second delay until\n\r");
}

void test_clock_client_task() {

  int tid   = my_tid();
  int p_tid = my_parent_tid();
  Message msg{};
  msg.type = MessageType::FUT_CLIENT_PARAMS;
  Message rcv_msg{};

  int rcv_len = send(p_tid, msg, rcv_msg);

  _assert(rcv_msg.type == MessageType::FUT_CLIENT_PARAMS_REPLY &&
              rcv_len == static_cast<int>(sizeof(rcv_msg)),
          "clock client task did not receive param msg");

  int cs_tid       = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  auto delay_ticks = rcv_msg.data.fut_params.delay_ticks;
  auto delay_count = rcv_msg.data.fut_params.delay_count;

  for (int delays_complete = 0; delays_complete < delay_count;
       ++delays_complete) {
    [[maybe_unused]] auto time = Delay(cs_tid, delay_ticks);
    uart_printf(CONSOLE,
                "T %d delay_ticks: %d delay_count: %d delays_complete: %d\n\r",
                tid, delay_ticks, delay_count, delays_complete);
  }
}
