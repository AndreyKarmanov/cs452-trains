#include "rx_server.h"
#include "message.h"
#include "syscall.h"
#include "uart.h"

static void rx_notifier_task() {
  int rx_tid = WhoIs(RX_Server::RX_SERVER_NAME);
  _assert(rx_tid >= 0, "RX SERVER WHOIS FAILED");

  while (true) {
    await_event(Event::UART_RX_IRQ);
    auto rcv_msg = send<RX::InterruptReplyMsg>(rx_tid, RX::InterruptMsg{});
    _assert(rcv_msg.has_value(), "RX INTERRUPT FAILED");
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

  reply(waiting_getc_tid, RX::GetcReplyMsg{.c = c.value()});
  waiting_getc_tid = -1;
}

void RX_Server::handle(const int tid, const RX::InterruptMsg &) {
  while (can_receive_io()) {
    _assert(rx_buffer.push(getc()), "RX SERVER: BUFFER FULL");
  }

  clear_uart_interrupt(UARTInterruptType::RXIM);
  clear_uart_interrupt(UARTInterruptType::RTIM);

  reply(tid, RX::InterruptReplyMsg{});

  try_reply_getc();
}

void RX_Server::handle(const int tid, const RX::GetcMsg &) {
  if (!rx_buffer.is_empty()) {
    auto c = rx_buffer.pop();
    _assert(c.has_value(), "RX SERVER: GETC POP FAILED");

    reply(tid, RX::GetcReplyMsg{.c = c.value()});
  } else {
    waiting_getc_tid = tid;
  }
}

void RX_Server::run() {
  int tid;
  Message msg{};
  receive(&tid, msg);
  std::visit([&](auto &&arg) { handle(tid, arg); }, msg);
}
