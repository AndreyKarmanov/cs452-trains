#include "rx_server.h"
#include "message.h"
#include "syscall.h"
#include "uart_new.h"

static void rx_notifier_task() {
  int rx_tid = WhoIs(RX_Server::RX_SERVER_NAME);
  _assert(rx_tid >= 0, "RX SERVER WHOIS FAILED");

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

void RX_Server::try_reply_getc() {
  if (waiting_getc_tid < 0 || rx_buffer.is_empty()) {
    return;
  }

  auto c = rx_buffer.pop();
  _assert(c.has_value(), "RX SERVER: GETC POP FAILED");

  Message reply_msg;
  reply_msg.type                 = MessageType::RX_GETC_REPLY;
  reply_msg.data.rx_getc_reply.c = c.value();
  reply(waiting_getc_tid, reply_msg);
  waiting_getc_tid = -1;
}

void RX_Server::run() {
  int tid;
  Message msg;
  auto rcv_size = receive(&tid, msg);
  _assert(rcv_size == static_cast<int>(sizeof(msg)),
          "RX SERVER: RECEIVED LESS THAN MSG");

  switch (msg.type) {
  case MessageType::RX_INTERRUPT: {
    while (can_receive_io()) {
      _assert(rx_buffer.push(getc()), "RX SERVER: BUFFER FULL");
    }

    clear_uart_interrupt(UARTInterruptType::RXIM);
    clear_uart_interrupt(UARTInterruptType::RTIM);

    Message reply_msg;
    reply_msg.type = MessageType::RX_INTERRUPT_REPLY;
    reply(tid, reply_msg);

    try_reply_getc();
    break;
  }

  case MessageType::RX_GETC: {
    if (!rx_buffer.is_empty()) {
      auto c = rx_buffer.pop();
      _assert(c.has_value(), "RX SERVER: GETC POP FAILED");

      Message reply_msg;
      reply_msg.type                 = MessageType::RX_GETC_REPLY;
      reply_msg.data.rx_getc_reply.c = c.value();
      reply(tid, reply_msg);
    } else {
      waiting_getc_tid = tid;
    }
    break;
  }

  default: {
    _assert(false, "RX SERVER: UNEXPECTED MESSAGE TYPE");
    break;
  }
  }
}
