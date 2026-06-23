#include "train_control.h"
#include "io_helpers.h"
#include "message.h"
#include "uart_tx_server.h"

template <> void TrainControlServer<>::tx_can_worker() {
  auto tc_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tc_tid >= 0, "TC SERVER WHOIS FAILED");

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  while (true) {
    auto tx_msg = send<TC::TX>(tc_tid, TC::TXReady{});
    if (!tx_msg.has_value()) {
      break;
    }

    auto frame = encode_frame(tx_msg->mrk);
    await_event(Event::CAN_TX_IRQ);
    tx_can(frame);
  }

  Offset_Puts(tx_tid, 2, "tx can worker EXITING\n\r");
}

template <> void TrainControlServer<>::rx_can_worker() {
  auto can_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");
  CANFRAME frame{};

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  while (true) {
    await_event(Event::CAN_RX_IRQ);
    rx_can(frame);

    auto rcv_msg = send<TC::Ack>(can_tid, TC::RX{.frame = frame});
    if (!rcv_msg.has_value()) {
      break;
    }
  }

  Offset_Puts(tx_tid, 3, "rx can worker EXITING\n\r");
}
