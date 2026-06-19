#include "clock_server.h"
#include "io_helpers.h"
#include "message.h"
#include "syscall.h"
#include "tx_server.h"

template <> void ClockServer<>::clock_tick_task() {
  int cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);

  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  Printf(tx_tid, "STARTED CLOCK NOTIFIER");

  while (true) {
    await_event(Event::CLOCK_TICK_1MS);
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
    yield();
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
  auto cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);

  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);

  auto time = Time(cs_tid);
  Printf(tx_tid, "Current time: %d ticks\n\r", time);

  time = Delay(cs_tid, 500);
  Printf(tx_tid, "5 second delay finished at %d ticks\n\r", time);

  time = DelayUntil(cs_tid, time + 500);
  Printf(tx_tid, "5 second delay until finished at %d ticks\n\r", time);
}

void test_clock_client_task() {

  int tid      = my_tid();
  int p_tid    = my_parent_tid();
  int tx_tid   = WhoIs(TX_Server::TX_SERVER_NAME);
  auto rcv_msg = send<FUT::ClientInitMsg>(p_tid, FUT::ClientParamRequestMsg{});

  _assert(rcv_msg.has_value(), "clock client task did not receive param msg");

  int cs_tid       = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  auto delay_ticks = rcv_msg->delay_ticks;
  auto delay_count = rcv_msg->delay_count;

  for (int delays_complete = 0; delays_complete < delay_count;
       ++delays_complete) {
    auto time = Delay(cs_tid, delay_ticks);
    Printf(tx_tid,
           "T %d delay_ticks: %d delay_count: %d delays_complete: %d at %d\n\r",
           tid, delay_ticks, delay_count, delays_complete, time);
  }
}
