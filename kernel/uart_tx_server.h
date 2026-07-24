#pragma once

#include "buffer.h"
#include "debug.h"
#include "message.h"
#include "name_server.h"
#include "syscall.h"
#include "uart.h"

struct UART0_TX_Traits {
  static constexpr const char *NAME = "TXSERVER";
  static constexpr size_t LINE      = CONSOLE;
};

struct UART3_TX_Traits {
  static constexpr const char *NAME = "TXSERVER3";
  static constexpr size_t LINE      = WEBSERIAL;
};

template <typename Traits> class UART_TX_Server_T {
public:
  static constexpr auto NAME = Traits::NAME;

  UART_TX_Server_T() {
    auto response = RegisterAs(NAME);
    _assert(response == 0, "TX SERVER REGISTERAS FAILED");
  }

  static constexpr size_t TX_BUFFER_SIZE = 20480;
  Buffer<char, TX_BUFFER_SIZE> tx_buffer;
  int notifier_tid;
  bool can_reply_to_notifier = false;
  bool buffer_has_pending_tx = false;

  void drain() {
    while (!tx_buffer.empty()) {
      if (!can_transmit_io(Traits::LINE)) {
        break;
      }
      auto c = tx_buffer.pop();
      putc(c.value(), Traits::LINE);
    }

    buffer_has_pending_tx = !tx_buffer.empty();
  }

  void reply_to_notifier() {
    can_reply_to_notifier = false;
    reply(notifier_tid, TX::ReplyMsg{});
  }

  void run() {
    int tid;
    Message msg{};
    receive(&tid, msg);

    std::visit([&](auto &&arg) { handle(tid, arg); }, msg);
  }

private:
  void handle(const int tid, const TX::SendMsg &msg) {
    bool overflowed = false;
    for (int i = 0; i < msg.len; ++i) {
      if (!tx_buffer.push(msg.data[i]) && !overflowed) {
        overflowed = true;
        debug_puts(CONSOLE, "FAIL: TX buffer overflow\n\r");
      }
    }
    drain();

    // reply to sender
    reply(tid, TX::ReplyMsg{});

    // conditionally reply to notifier
    if (can_reply_to_notifier && buffer_has_pending_tx) {
      reply_to_notifier();
    }
  }

  void handle(const int tid, const TX::InterruptMsg &) {
    notifier_tid          = tid;
    can_reply_to_notifier = true;
    drain();

    // conditionally reply to notifier
    // we do this as unblocking the notifier means an exception will be
    // immediately raised
    if (can_reply_to_notifier && buffer_has_pending_tx) {
      reply_to_notifier();
    }
  }

  template <class T> void handle(int tid, const T &) { reply_with_error(tid); }
};

using UART_TX_Server   = UART_TX_Server_T<UART0_TX_Traits>;
using UART03_TX_Server = UART_TX_Server_T<UART3_TX_Traits>;

void uart_tx_server_task();
void uart03_tx_server_task();
