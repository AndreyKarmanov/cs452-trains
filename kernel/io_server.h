#pragma once

#include "buffer.h"
#include "debug.h"
#include "name_server.h"

class IO_Server {
public:
  static constexpr auto IO_SERVER_NAME = "IOSERVER";

  IO_Server() {
    auto response = RegisterAs(IO_SERVER_NAME);
    _assert(response == 0, "IO SERVER REGISTERAS FAILED");
  }

  static constexpr size_t TX_BUFFER_SIZE = 1024;
  Buffer<char, TX_BUFFER_SIZE> tx_buffer;

  void run();
};

void io_server_task();
