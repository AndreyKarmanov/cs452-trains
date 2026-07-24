#include "clock_server.h"
#include "message.h"
#include "syscall.h"
#include "uart_tx_server.h"

template <> void ClockServer<>::clock_tick_task() {
  int cs_tid = WhoIs(ClockServer<>::NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  while (true) {
    await_event(Event::CLOCK_TICK);
    auto rcv_msg = send<CS::TickMsg>(cs_tid, CS::TickMsg{});
    if (!rcv_msg.has_value()) {
      break;
    }
  }
}

void clock_server_task() {
  ClockServer<> clock_server;
  while (true) {
    clock_server.run();
  }
}

int Time(int tid) {
  auto res = send<CS::TimeReplyMsg>(tid, CS::TimeMsg{});
  if (!res) {
    return res.error();
  }
  return res->ticks;
}

int Delay(int tid, uint32_t ticks) {
  auto res = send<CS::DelayReplyMsg>(tid, CS::DelayMsg{.ticks = ticks});
  if (!res) {
    return res.error();
  }
  return res->ticks;
}

int DelayUntil(int tid, uint32_t ticks) {
  auto res = send<CS::DelayReplyMsg>(tid, CS::DelayUntilMsg{.ticks = ticks});
  if (!res) {
    return res.error();
  }
  return res->ticks;
}

void test_clock_server() {
  auto cs_tid = WhoIs(ClockServer<>::NAME);

  int tx_tid = WhoIs(UART_TX_Server::NAME);

  auto time = Time(cs_tid);
  Puts(tx_tid, "Current time: ", time, " ticks\n\r");

  time = Delay(cs_tid, 500);
  Puts(tx_tid, "5 second delay finished at ", time, " ticks\n\r");

  time = DelayUntil(cs_tid, time + 500);
  Puts(tx_tid, "5 second delay until finished at ", time, " ticks\n\r");
}

void test_clock_client_task() {

  int tid      = my_tid();
  int p_tid    = my_parent_tid();
  int tx_tid   = WhoIs(UART_TX_Server::NAME);
  auto rcv_msg = send<FUT::ClientInitMsg>(p_tid, FUT::ClientParamRequestMsg{});

  _assert(rcv_msg.has_value(), "clock client task did not receive param msg");

  int cs_tid       = WhoIs(ClockServer<>::NAME);
  auto delay_ticks = rcv_msg->delay_ticks;
  auto delay_count = rcv_msg->delay_count;

  for (int delays_complete = 0; delays_complete < delay_count;
       ++delays_complete) {
    auto time = Delay(cs_tid, delay_ticks);
    Puts(tx_tid, "T ", tid, " delay_ticks: ", delay_ticks,
         " delay_count: ", delay_count, " delays_complete: ", delays_complete,
         " at ", time, "\n\r");
  }
}
