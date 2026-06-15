#pragma once

#include "buffer.h"
#include "debug.h"
#include "name_server.h"

class TX_Server {
public:
  static constexpr auto TX_SERVER_NAME = "TXSERVER";

  TX_Server() {
    auto response = RegisterAs(TX_SERVER_NAME);
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
};

void tx_server_task();

int Putc(int tid, unsigned char c);
