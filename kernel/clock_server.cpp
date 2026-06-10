#pragma once

#include "message.h"
#include "syscall.h"

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
