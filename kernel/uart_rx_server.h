#pragma once

#include "buffer.h"
#include "debug.h"
#include "name_server.h"

class UART_RX_Server {
public:
  static constexpr auto RX_SERVER_NAME   = "RXSERVER";
  static constexpr size_t RX_BUFFER_SIZE = 1024;

  UART_RX_Server() {
    auto response = RegisterAs(RX_SERVER_NAME);
    _assert(response == 0, "RX SERVER REGISTERAS FAILED");
  }

  Buffer<char, RX_BUFFER_SIZE> rx_buffer;
  int waiting_getc_tid = -1;

  void try_reply_getc();
  void run();

private:
  void handle(const int tid, const RX::InterruptMsg &);
  void handle(const int tid, const RX::GetcMsg &);
  template <class T> void handle(int tid, const T &) { reply_with_error(tid); }
};

void uart_rx_server_task();
