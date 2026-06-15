#include "tx_server.h"
#include "io_helpers.h"
#include "syscall.h"
#include "uart.h"

static void tx_notifier_task() {
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  Printf(tx_tid, "STARTED IO NOTIFIER");

  while (true) {
    await_event(Event::UART_TX_IRQ);

    // unlike other notifiers, we want this to block as the await_event init is
    // what unmasks the interrupt
    auto rcv_msg = send<TX::ReplyMsg>(tx_tid, TX::InterruptMsg{});
    _assert(rcv_msg.has_value(), "TX INTERRUPT FAILED");
  }
}

void tx_server_task() {
  TX_Server tx_server;
  create(2, tx_notifier_task);
  while (true) {
    tx_server.run();
    yield();
  }
}

void TX_Server::drain() {
  while (!tx_buffer.is_empty()) {
    if (!can_transmit_io()) {
      break;
    }
    auto c = tx_buffer.pop();
    putc(c.value());
  }

  buffer_has_pending_tx = !tx_buffer.is_empty();
}

void TX_Server::reply_to_notifier() {
  can_reply_to_notifier = false;
  reply(notifier_tid, TX::ReplyMsg{});
}

void TX_Server::handle(const int tid, const TX::SendMsg &msg) {
  for (int i = 0; i < msg.len; ++i) {
    tx_buffer.push(msg.data[i]);
  }
  drain();

  // reply to sender
  reply(tid, TX::ReplyMsg{});

  // conditionally reply to notifier
  if (can_reply_to_notifier && buffer_has_pending_tx) {
    reply_to_notifier();
  }
}

void TX_Server::handle(const int tid, const TX::InterruptMsg &) {
  notifier_tid          = tid;
  can_reply_to_notifier = true;
  drain();

  // conditionally reply to notifier
  // we do this as unblocking the notifier means an exception will be
  // immediately raised
  if (can_reply_to_notifier && buffer_has_pending_tx) {
    reply_to_notifier();
  }
}

void TX_Server::run() {
  int tid;
  Message msg{};
  receive(&tid, msg);

  std::visit([&](auto &&arg) { handle(tid, arg); }, msg);
}
