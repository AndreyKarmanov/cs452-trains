#include "uart03_tx_server.h"
#include "syscall.h"
#include "uart.h"

static void tx03_notifier_task() {
  int tx_tid = WhoIs(UART03_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER3 WHOIS FAILED");

  while (true) {
    await_event(Event::UART3_TX_IRQ);

    // unlike other notifiers, we want this to block as the await_event init is
    // what unmasks the interrupt
    auto rcv_msg = send<TX::ReplyMsg>(tx_tid, TX::InterruptMsg{});
    _assert(rcv_msg.has_value(), "TX3 INTERRUPT FAILED");
  }
}

void uart03_tx_server_task() {
  UART03_TX_Server uart03_tx_server;
  create(2, tx03_notifier_task);
  while (true) {
    uart03_tx_server.run();
    yield();
  }
}

void UART03_TX_Server::drain() {
  while (!tx_buffer.empty()) {
    if (!can_transmit_io(WEBSERIAL)) {
      break;
    }
    auto c = tx_buffer.pop();
    putc(c.value(), WEBSERIAL);
  }

  buffer_has_pending_tx = !tx_buffer.empty();
}

void UART03_TX_Server::reply_to_notifier() {
  can_reply_to_notifier = false;
  reply(notifier_tid, TX::ReplyMsg{});
}

void UART03_TX_Server::handle(const int tid, const TX::SendMsg &msg) {
  bool overflowed = false;
  for (int i = 0; i < msg.len; ++i) {
    if (!tx_buffer.push(msg.data[i]) && !overflowed) {
      overflowed = true;
      debug_puts(CONSOLE, "FAIL: TX3 buffer overflow\n\r");
    }
  }
  drain();

  // reply to sender
  reply(tid, TX::ReplyMsg{});

  // conditionally reply to notifier
  if (can_reply_to_notifier && buffer_has_pending_tx) {
    reply_to_notifier();
  }
}

void UART03_TX_Server::handle(const int tid, const TX::InterruptMsg &) {
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

void UART03_TX_Server::run() {
  int tid;
  Message msg{};
  receive(&tid, msg);

  std::visit([&](auto &&arg) { handle(tid, arg); }, msg);
}
