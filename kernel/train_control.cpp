#include "train_control.h"
#include "can_server.h"

template <> void TrainControlServer<>::tx_can_worker() {
  auto tc_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tc_tid >= 0, "TC SERVER WHOIS FAILED");

  auto can_tid = WhoIs(CanServer<>::CAN_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  while (true) {
    auto tx_msg = send<TC::TX>(tc_tid, TC::TXReady{});
    if (!tx_msg.has_value()) {
      break;
    }

    auto frame      = encode_frame(tx_msg->mrk);
    int send_result = tx_msg->delay_ticks > 0
                          ? CanDelay(can_tid, frame, tx_msg->delay_ticks)
                          : CanSend(can_tid, frame);
    _assert(send_result == 0, "CAN COURRIER SEND FAILED");
  }
}

template <> void TrainControlServer<>::rx_can_worker() {
  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  CANFRAME frame{};
  TC::RX msg{};
  while (true) {
    await_event(Event::CAN_RX_IRQ);
    rx_can(frame);
    msg.mrk         = decode_frame(frame);
    auto cans_reply = send<TC::Ack>(tcs_tid, msg);
    if (!cans_reply.has_value()) {
      break;
    }
  }
}
