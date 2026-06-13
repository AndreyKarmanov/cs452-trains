#include "rx_server.h"
#include "uart_new.h"

static void rx_notifier_task() {
  int rx_tid = WhoIs(RX_Server::RX_SERVER_NAME);
  _assert(rx_tid >= 0, "RX SERVER WHOIS FAILED");

  // uart_printf(CONSOLE, "STARTED IO NOTIFIER");

  Message msg;
  msg.type = MessageType::RX_INTERRUPT;
  Message rcv_msg;

  while (true) {
    await_event(Event::UART_RX_IRQ);
    auto rcv_len = send(rx_tid, msg, rcv_msg);
    _assert(rcv_len >= 0, "RX INTERRUPT FAILED");
  }
}

void rx_server_task() {
  RX_Server rx_server;
  create(2, rx_notifier_task);
  while (true) {
    rx_server.run();
    yield();
  }
}

void RX_Server::run() {
  int tid;
  Message msg;
  auto rcv_size = receive(&tid, msg);
  _assert(rcv_size == static_cast<int>(sizeof(msg)),
          "RX SERVER: RECEIVED LESS THAN MSG");

  _assert(msg.type == MessageType::RX_INTERRUPT,
          "RX SERVER: UNEXPECTED MESSAGE TYPE");

  while (can_receive_io()) {
    char c = getc();
    // do stuff with char
    // todo; send to tx buffer
  }

  // clear interrupt icr after reading rx
  clear_uart_interrupt(UARTInterruptType::RXIM);
  clear_uart_interrupt(UARTInterruptType::RTIM);

  // reply
  Message reply_msg;
  reply_msg.type = MessageType::RX_INTERRUPT_REPLY;
  reply(tid, reply_msg);
}
