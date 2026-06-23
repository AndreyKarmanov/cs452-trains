#pragma once

#include "io_helpers.h"
#include "train_cli_worker.h"
#include "train_control.h"
#include "train_ui_worker.h"
#include "uart_tx_server.h"

static void train_control_server() {
  TrainControlServer<> server;
  for (;;) {
    server.run();
  }
}

inline void train_controller_program_task() {
  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");
  Puts(tx_tid, "\033[2J\033[1;1H");

  create(2, train_control_server);
  create(3, ui_update_worker);
  auto train_ui_tid = create(3, cli_worker);

  await_task(train_ui_tid);
  Debug_Puts(tx_tid, "train_controller_program_task EXITING\n\r");
}
