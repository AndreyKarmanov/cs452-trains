#include "uart_tx_server.h"
#include "syscall.h"
#include "message.h"
#include "syscall.h"

template <typename ServerT, Event IRQ_EVENT> static void tx_notifier_task() {
  int tx_tid = WhoIs(ServerT::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  while (true) {
    await_event(IRQ_EVENT);

    // unlike other notifiers, we want this to block as the await_event init is
    // what unmasks the interrupt
    auto rcv_msg = send<TX::ReplyMsg>(tx_tid, TX::InterruptMsg{});
    _assert(rcv_msg.has_value(), "TX INTERRUPT FAILED");
  }
}

template <typename ServerT, Event IRQ_EVENT>
static void uart_tx_server_task_impl() {
  ServerT uart_tx_server;
  create(2, tx_notifier_task<ServerT, IRQ_EVENT>);
  while (true) {
    uart_tx_server.run();
    yield();
  }
}

void uart_tx_server_task() {
  uart_tx_server_task_impl<UART_TX_Server, Event::UART_TX_IRQ>();
}

void uart03_tx_server_task() {
  uart_tx_server_task_impl<UART03_TX_Server, Event::UART3_TX_IRQ>();
}

// tid should be the RX server tid
int Getc(int tid) {
  auto rcv_msg = send<RX::GetcReplyMsg>(tid, RX::GetcMsg{});
  if (!rcv_msg.has_value()) {
    return -1;
  }
  return static_cast<int>(rcv_msg->c);
}
