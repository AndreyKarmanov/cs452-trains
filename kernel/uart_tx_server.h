#pragma once

#include "buffer.h"
#include "debug.h"
#include "name_server.h"

class UART_TX_Server {
public:
  static constexpr auto NAME = "TXSERVER";

  UART_TX_Server() {
    auto response = RegisterAs(NAME);
    _assert(response == 0, "TX SERVER REGISTERAS FAILED");
  }

  static constexpr size_t TX_BUFFER_SIZE = 1024;
  Buffer<char, TX_BUFFER_SIZE> tx_buffer;
  int notifier_tid;
  bool can_reply_to_notifier = false;
  bool buffer_has_pending_tx = false;

  void drain();
  void reply_to_notifier();
  void run();

private:
  void handle(const int tid, const TX::SendMsg &);
  void handle(const int tid, const TX::InterruptMsg &);
  template <class T> void handle(int tid, const T &) { reply_with_error(tid); }
};

void uart_tx_server_task();
