#include "train_control.h"
#include "message.h"

template <> void TrainControlServer<>::tx_can_worker() {
  auto tc_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tc_tid >= 0, "TC SERVER WHOIS FAILED");

  while (true) {
    auto tx_msg = send<TC::TX>(tc_tid, TC::TXReady{});
    if (!tx_msg.has_value()) {
      break;
    }

    auto frame = encode_frame(tx_msg->mrk);
    await_event(Event::CAN_TX_IRQ);
    tx_can(frame);
  }
}

template <> void TrainControlServer<>::rx_can_worker() {
  auto can_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");
  CANFRAME frame{};

  while (true) {
    await_event(Event::CAN_RX_IRQ);
    rx_can(frame);

    auto rcv_msg = send<TC::Ack>(can_tid, TC::RX{.frame = frame});
    if (!rcv_msg.has_value()) {
      break;
    }
  }
}
