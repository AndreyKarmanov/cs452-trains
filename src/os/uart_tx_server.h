#pragma once

#include "buffer.h"
#include "debug.h"
#include "message.h"
#include "name_server.h"
#include "syscall.h"
#include "uart.h"
#include "message.h"
#include "static_string.h"
#include "syscall.h"
#include <cstring>


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


int Getc(int tid);

template <size_t SIZE> int Puts(int tid, const StaticString<SIZE> &str) {
  size_t offset    = 0;
  size_t total_len = str.len;

  while (offset < total_len) {
    size_t chunk = total_len - offset;
    if (chunk > static_cast<size_t>(TX::MAX_DATA_LENGTH)) {
      chunk = TX::MAX_DATA_LENGTH;
    }

    TX::SendMsg send_msg{};
    send_msg.len = static_cast<int>(chunk);
    std::memcpy(send_msg.data, str.data + offset, chunk);
    auto rcv_msg = send<TX::ReplyMsg>(tid, send_msg);
    if (!rcv_msg.has_value()) {
      return -1;
    }

    offset += chunk;
  }

  return 0;
}

template <size_t SIZE> int Debug_Puts(int tid, const StaticString<SIZE> &str) {
  static constexpr int DEBUG_LINE_START = 40;
  static int debug_scroll_line          = 0;

  const int row = DEBUG_LINE_START + debug_scroll_line;
#if !defined(DATA_COLLECTION) || !DATA_COLLECTION
  debug_scroll_line = (debug_scroll_line + 1) % 40;
#else
  debug_scroll_line = (debug_scroll_line + 1);
#endif
  StaticString<TX::MAX_DATA_LENGTH> out;
  out.set("\033[s\033[", row, ";1H\033[K", str, "\n\r\033[K\033[u");
  return Puts(tid, out);
}

template <typename... Args> int Debug_Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Debug_Puts(tid, str);
}

template <size_t SIZE, typename T>
bool AppendPadded(StaticString<SIZE> &str, const T &value, size_t width,
                  char fill = ' ') {
  StaticString<SIZE> value_str;
  value_str.append(value);

  if (value_str.len >= width) {
    return str.append(value_str);
  }

  for (size_t i = value_str.len; i < width; ++i) {
    if (!str.append(fill)) {
      return false;
    }
  }

  return str.append(value_str);
}

template <typename... Args>
int Offset_Puts(int tid, int offset, const Args &...args) {
  static constexpr int DEBUG_LINE = 40;
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set("\033[s\033[", DEBUG_LINE + offset, ";1H", args..., "\033[u");
  return Puts(tid, str);
}

template <typename... Args> int Puts(int tid, const Args &...args) {
  StaticString<TX::MAX_DATA_LENGTH> str;
  str.set(args...);
  return Puts(tid, str);
}

