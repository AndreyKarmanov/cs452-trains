#include "tx_server.h"
#include "syscall.h"
#include "uart_new.h"

static void tx_notifier_task() {
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  // uart_printf(CONSOLE, "STARTED IO NOTIFIER");

  Message msg;
  msg.type = MessageType::TX_INTERRUPT;
  Message rcv_msg;

  while (true) {
    await_event(Event::UART_TX_IRQ);

    // unlike other notifiers, we want this to block as the await_event init is
    // what unmasks the interrupt
    auto rcv_len = send(tx_tid, msg, rcv_msg);
    _assert(rcv_len >= 0, "TX INTERRUPT FAILED");
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
  Message notifier_msg{};
  notifier_msg.type     = MessageType::TX_REPLY;
  can_reply_to_notifier = false;
  reply(notifier_tid, notifier_msg);
}

void TX_Server::run() {
  int tid;
  Message msg;
  auto rcv_size = receive(&tid, msg);
  _assert(rcv_size == static_cast<int>(sizeof(msg)),
          "TX SERVER: RECEIVED LESS THAN MSG");

  switch (msg.type) {
  case MessageType::TX_SEND: {
    auto &tx_send = msg.data.tx_send;
    for (int i = 0; i < tx_send.len; ++i) {
      tx_buffer.push(tx_send.data[i]);
    }
    drain();

    // reply to sender
    Message msg{};
    msg.type = MessageType::TX_REPLY;
    reply(tid, msg);

    // conditionally reply to notifier
    if (can_reply_to_notifier && buffer_has_pending_tx) {
      reply_to_notifier();
    }

    break;
  }
  case MessageType::TX_INTERRUPT: {
    notifier_tid          = tid;
    can_reply_to_notifier = true;
    drain();

    // conditionally reply to notifier
    // we do this as unblocking the notifier means an exception will be
    // immediately raised
    if (can_reply_to_notifier && buffer_has_pending_tx) {
      reply_to_notifier();
    }
    break;
  }
  default: {
    break;
  }
  }
}

int Putc(int tid, unsigned char c) {
  Message msg;
  msg.type                 = MessageType::TX_SEND;
  msg.data.tx_send.len     = 1;
  msg.data.tx_send.data[0] = c;
  Message rcv_msg;
  auto rcv_len = send(tid, msg, rcv_msg);

  return rcv_len < 0 ? -1 : 0;
}

static void tx_client_task() {
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  while (true) {
    await_event(Event::UART_TX_IRQ);
  }
}
