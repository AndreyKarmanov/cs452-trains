#pragma once

#include "debug.h"
#include "name_server.h"

class RX_Server {
public:
  static constexpr auto RX_SERVER_NAME = "RXSERVER";

  RX_Server() {
    auto response = RegisterAs(RX_SERVER_NAME);
    _assert(response == 0, "RX SERVER REGISTERAS FAILED");
  }

  void run();
};

void rx_server_task();
