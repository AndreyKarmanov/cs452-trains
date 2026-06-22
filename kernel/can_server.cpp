#include "can_server.h"
#include "can_rx_server.h"
#include "clock_server.h"

template <> void CanServer<>::tx_can_worker() {
  auto can_tid = WhoIs(CanServer<>::CAN_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  auto can_rx_tid = WhoIs(CAN_RxServer::CAN_RX_SERVER_NAME);
  _assert(can_rx_tid >= 0, "CAN RX SERVER WHOIS FAILED");

  while (true) {
    // get next frame to send
    auto rcv_msg = send<CAN::TXMsg>(can_tid, CAN::TXReadyMsg{});
    if (!rcv_msg.has_value()) {
      break;
    }

    // notify can receiver of message that will be sent
    auto pace_register = send<CRX::AckMsg>(
        can_rx_tid, CRX::PaceRegisterMsg{.frame = rcv_msg->frame});
    _assert(pace_register.has_value(), "CAN PACE REGISTER FAILED");

    await_event(Event::CAN_TX_IRQ);
    tx_can(rcv_msg->frame);

    // wait for can receiver to ack sent message
    auto pace_await = send<CRX::AckMsg>(can_rx_tid, CRX::PaceAwaitMsg{});
    _assert(pace_await.has_value(), "CAN PACE AWAIT FAILED");
  }
}

template <> void CanServer<>::tick_can_worker() {
  int can_tid = WhoIs(CanServer<>::CAN_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  int cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");
  auto curr_tick = static_cast<uint32_t>(Time(cs_tid));

  while (true) {
    curr_tick    = DelayUntil(cs_tid, curr_tick + 10);
    auto rcv_msg = send<CAN::AckMsg>(can_tid, CAN::TickMsg{curr_tick});
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
