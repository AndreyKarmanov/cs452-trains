#include "io_server.h"
#include "uart_new.h"

static void io_notifier_task() {
  int io_tid = WhoIs(IO_Server::IO_SERVER_NAME);
  _assert(io_tid >= 0, "IO SERVER WHOIS FAILED");

  // uart_printf(CONSOLE, "STARTED IO NOTIFIER");

  Message msg;
  msg.type = MessageType::IO_INTERRUPT;
  Message rcv_msg;

  while (true) {
    await_event(Event::UART_IRQ);
    auto rcv_len = send(io_tid, msg, rcv_msg);
    _assert(rcv_len >= 0, "IO INTERRUPT FAILED");
  }
}

void io_server_task() {
  IO_Server io_server;
  create(2, io_notifier_task);
  while (true) {
    io_server.run();
    yield();
  }
}

void IO_Server::run() {
  int tid;
  Message msg;
  auto rcv_size = receive(&tid, msg);
  _assert(rcv_size == static_cast<int>(sizeof(msg)),
          "IO SERVER: RECEIVED LESS THAN MSG");

  _assert(msg.type == MessageType::IO_INTERRUPT,
          "IO SERVER: UNEXPECTED MESSAGE TYPE");

  // determine the execption type
  // best: check status → action or wait for interrupt → repeat
  // avoid interrupt handling as much as possible

  // read UART MIS register(s) to find out which UART interrupt(s)
  // level signal: disable interrupt at IMSC
  // clear interrupt using UART ICR
}

/*
We want to send and receive without blocking each other.

Optimal send/receive pattern is:
s/r until cannot, then self unmask and put to sleep.

Maybe I should have a transmit server and a receive server.
but then, as the message passes, this is already another context switch.

Maybe keep static variables in io_server that determines if can send or receive?
But then, we really want to be put to sleep. meaning, we have to use await
event.

So maybe, instead of 1 io server, we have 1 tx server and 1 rx server?
rx related items get send there?

*/