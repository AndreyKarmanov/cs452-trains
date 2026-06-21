#include "can_server.h"
#include "clock_server.h"

template <> void CanServer<>::tx_can_worker() {
  auto can_tid = WhoIs(CanServer<>::CAN_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  while (true) {
    auto rcv_msg = send<CAN::TXMsg>(can_tid, CAN::TXReadyMsg{});
    if (!rcv_msg.has_value()) {
      break;
    }

    await_event(Event::CAN_TX_IRQ);
    tx_can(rcv_msg->frame);
  }
}

template <> void CanServer<>::tick_can_worker() {
  int can_tid = WhoIs(CanServer<>::CAN_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  int cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");
  auto curr_tick = Time(cs_tid);

  while (true) {
    curr_tick    = DelayUntil(cs_tid, curr_tick + 10);
    auto rcv_msg = send<CAN::AckMsg>(can_tid, CAN::TickMsg{});
    if (!rcv_msg.has_value()) {
      break;
    }
  }
}

void can_server_task() {
  CanServer<> can_server;
  while (true) {
    can_server.run();
    yield();
  }
}

int CanSend(int tid, const CANFRAME &frame) {
  auto res = send<CAN::AckMsg>(tid, CAN::SendMsg{.frame = frame});
  if (!res) {
    return res.error();
  }
  return 0;
}

int CanDelay(int tid, const CANFRAME &frame, uint32_t delay_ticks) {
  auto res = send<CAN::AckMsg>(
      tid, CAN::DelayMsg{.frame = frame, .delay_ticks = delay_ticks});
  if (!res) {
    return res.error();
  }
  return 0;
}

int CanDelayUntil(int tid, const CANFRAME &frame, uint32_t ticks) {
  auto res = send<CAN::AckMsg>(
      tid, CAN::DelayUntilMsg{.frame = frame, .ticks = ticks});
  if (!res) {
    return res.error();
  }
  return 0;
}
