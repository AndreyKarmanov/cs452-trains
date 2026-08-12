#pragma once

#include "debug.h"
#include "message.h"
#include "mrk.h"
#include <cstddef>
#include <expected>

enum class Syscall {
  CREATE          = 0,
  MY_TID          = 1,
  MY_PARENT_TID   = 2,
  YIELD           = 3,
  EXIT            = 4,
  SEND            = 5,
  RECEIVE         = 6,
  REPLY           = 7,
  AWAIT_EVENT     = 8,
  EMIT_EVENT      = 9,
  PARK            = 10,
  KERNEL_IDLE_PCT = 11,
  TX_CAN          = 12,
  RX_CAN          = 13,
};

// make sure that event count is the last event!!
// this is pivotal to ensure we can use it as the number of events :)
enum class Event {
  CLOCK_TICK,
  DELAY_5S,
  UART_RX_IRQ,
  UART_TX_IRQ,
  UART3_TX_IRQ,
  CAN_RX_IRQ,
  CAN_TX_IRQ,
  TASK_EXIT,
  EVENT_COUNT
};
constexpr auto TOTAL_EVENT_TYPES = static_cast<size_t>(Event::EVENT_COUNT);

int create(int priority, void (*function)());
int my_tid();
int my_parent_tid();
void yield();
void exit();
void park();
int kernel_idle_pct();

int send(int tid, const char *msg, int msglen, char *reply, int rplen);
template <typename T> std::expected<T, int> send(int tid, Message msg) {
  Message reply_msg;

  auto rcv_len = send(tid, reinterpret_cast<const char *>(&msg), sizeof(msg),
                      reinterpret_cast<char *>(&reply_msg), sizeof(reply_msg));

  if (rcv_len < 0) {
    return std::unexpected(rcv_len);
  }

  if (auto *val_ptr = std::get_if<T>(&reply_msg)) {
    return *val_ptr;
  }

  return std::unexpected(-2);
}

void receive(int *tid, Message &msg);
int receive(int *tid, char *msg, int msglen);

void reply(int tid, const Message &msg);
int reply(int tid, const char *reply, int rplen);
void reply_with_error(int tid, int error_code = 0);

void await_task(int tid);
int await_event(Event event);
void emit_event(Event event);

bool tx_can(const CANFRAME &frame);
bool rx_can(CANFRAME &frame);