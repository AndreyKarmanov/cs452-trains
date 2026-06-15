#include "clock_server.h"
#include "io_helpers.h"
#include "message.h"
#include "syscall.h"
#include "tx_server.h"

static void clock_notifier_task() {
  int cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  int tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);

  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  Printf(tx_tid, "STARTED CLOCK NOTIFIER");

  Message msg;
  msg.type = MessageType::CS_TICK;
  Message rcv_msg;

  while (true) {
    await_event(Event::CLOCK_TICK_1MS);
    auto rcv_len = send(cs_tid, msg, rcv_msg);
    _assert(rcv_len >= 0, "CLOCK TICK FAILED");
  }
}

void clock_server_task() {
  ClockServer<> clock_server;
  create(2, clock_notifier_task);
  while (true) {
    clock_server.run();
    yield();
  }
}

int Time(int tid) {
  Message msg;
  Message rcv_msg;
  msg.type = MessageType::CS_TIME;

  int rcv_len = send(tid, msg, rcv_msg);
  if (rcv_len < static_cast<int>(sizeof(rcv_msg)) ||
      rcv_msg.type != MessageType::CS_TIME_REPLY) {
    return -1;
  }
  return rcv_msg.data.cs_time_reply.ticks;
}

int Delay(int tid, int ticks) {
  Message msg;
  Message rcv_msg;
  msg.type                = MessageType::CS_DELAY;
  msg.data.cs_delay.ticks = ticks;
  int rcv_len             = send(tid, msg, rcv_msg);
  if (rcv_len < static_cast<int>(sizeof(rcv_msg)) ||
      rcv_msg.type != MessageType::CS_DELAY_REPLY) {
    return -1;
  }
  return rcv_msg.data.cs_delay_reply.ticks;
}

int DelayUntil(int tid, int ticks) {
  Message msg;
  Message rcv_msg;
  msg.type                      = MessageType::CS_DELAY_UNTIL;
  msg.data.cs_delay_until.ticks = ticks;
  int rcv_len                   = send(tid, msg, rcv_msg);
  if (rcv_len < static_cast<int>(sizeof(rcv_msg)) ||
      rcv_msg.type != MessageType::CS_DELAY_REPLY) {
    return -1;
  }
  return rcv_msg.data.cs_delay_reply.ticks;
}
