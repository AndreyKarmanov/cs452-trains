#pragma once

#include "message.h"
#include <cstddef>

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
  PARK            = 9,
  KERNEL_IDLE_PCT = 10,
};

// make sure that event count is the last event!!
// this is pivotal to ensure we can use it as the number of events :)
enum class Event {
  CLOCK_TICK_1MS,
  DELAY_5S,
  UART_RX_IRQ,
  UART_TX_IRQ,
  EVENT_COUNT
};
constexpr auto TOTAL_EVENT_TYPES = static_cast<size_t>(Event::EVENT_COUNT) + 1;

int create(int priority, void (*function)());
int my_tid();
int my_parent_tid();
void yield();
void exit();
void park();
int kernel_idle_pct();

int send(int tid, const Message msg, Message &reply_msg);
int send(int tid, const char *msg, int msglen, char *reply, int rplen);

int receive(int *tid, Message &msg);
int receive(int *tid, char *msg, int msglen);

int reply(int tid, Message msg);
int reply(int tid, const char *reply, int rplen);
int reply_with_error(int tid, int error_code = 0);

void await_task(int tid);
void await_event(Event event);