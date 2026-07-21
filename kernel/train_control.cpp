#include "train_control.h"
#include "clock_server.h"
#include "io_helpers.h"
#include "message.h"
#include "uart_tx_server.h"

template <> void TrainControlServer<>::tx_can_worker() {
  auto tc_tid = WhoIs(TrainControlServer<>::NAME);
  _assert(tc_tid >= 0, "TC SERVER WHOIS FAILED");

  auto tx_tid = WhoIs(UART_TX_Server::NAME);
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

  Debug_Puts(tx_tid, "tx can worker EXITING\n\r");
}

template <> void TrainControlServer<>::rx_can_worker() {
  auto can_tid = WhoIs(TrainControlServer<>::NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");
  CANFRAME frame{};

  auto tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto cs_tid = WhoIs(ClockServer<>::NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  while (true) {
    await_event(Event::CAN_RX_IRQ);
    auto time = static_cast<uint32_t>(Time(cs_tid));
    rx_can(frame);

    auto rcv_msg = send<TC::Ack>(can_tid, TC::RX{.frame = frame, .time = time});
    if (!rcv_msg.has_value()) {
      break;
    }
  }

  Debug_Puts(tx_tid, "rx can worker EXITING\n\r");
}

template <> void TrainControlServer<>::train_tick_worker() {
  auto can_tid = WhoIs(TrainControlServer<>::NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  auto tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto cs_tid = WhoIs(ClockServer<>::NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  auto time = static_cast<uint32_t>(Time(cs_tid));

  while (true) {
    time = static_cast<uint32_t>(
        DelayUntil(cs_tid, time + TICKS_BETWEEN_TRAIN_TICKS));

    auto rcv_msg = send<TC::Ack>(can_tid, TC::Tree::Tick{.time = time});
    if (!rcv_msg.has_value()) {
      break;
    }
  }

  Debug_Puts(tx_tid, "tick worker EXITING\n\r");
}
