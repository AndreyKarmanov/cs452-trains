#pragma once

#include "debug.h"
#include "io_helpers.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include "train_state.h"
#include "tx_server.h"
#include <cstddef>
#include <ctype.h>

uint32_t print_state(int tx_tid, const State &state);

template <size_t CLI_BUFFER_SIZE = 64> class TrainUIServer {
  StaticString<CLI_BUFFER_SIZE> buf{};
  Buffer<UserCmd, 8> cmd_buf;

  int tx_tid;

  static void ui_update_worker();
  static void cli_worker();
  static void command_worker();
  int waiting_command_worker_tid = -1;

  UserCmd parse_command();

  static constexpr auto CONSOLE_LINE = 3;

public:
  static constexpr auto TC_UI_SERVER_NAME = "TCUISERVER";
  TrainUIServer() {
    auto response = RegisterAs(TC_UI_SERVER_NAME);
    _assert(response == 0, "TC_UI_SERVER_NAME REGISTERAS FAILED");

    tx_tid = WhoIs(TX_Server::TX_SERVER_NAME);
    _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

    create(4, cli_worker);
    create(4, command_worker);
    create(5, ui_update_worker);
  }

  void handle(int sender_tid, const TC::CLIInput &msg) {
    auto c = msg.c;
    if (isprint(c)) {
      if (!buf.append(c)) {
        reply(sender_tid, TC::Ack{});
        return;
      }
      Puts(tx_tid, "\033[", CONSOLE_LINE, ";1H\033[K> ", buf, "\n\r");
    } else if ((c == 0x08 || c == 0x7f) && !buf.empty()) { // backspace
      buf.pop_back();
      Puts(tx_tid, "\033[", CONSOLE_LINE, ";1H\033[K> ", buf, "\n\r");
    } else if (c == '\r') { // enter
      auto result = parse_command();
      Puts(tx_tid, "\033[", CONSOLE_LINE, ";2H> ", buf, "\n\r");
      buf.clear();
      if (waiting_command_worker_tid >= 0 &&
          result.type != UserCmd::Type::Invalid) {
        reply(waiting_command_worker_tid, TC::CLICmd{result});
        waiting_command_worker_tid = -1;
      } else if (result.type != UserCmd::Type::Invalid) {
        auto pushed = cmd_buf.push(result);
        _assert(pushed, "COMMAND BUFFER FULL");
      }
    }

    reply(sender_tid, TC::Ack{});
  }

  void handle(int sender_tid, const TC::UIUpdate &msg) {
    print_state(tx_tid, msg.state);
    reply(sender_tid, TC::Ack{});
  }

  void handle(int sender_tid, const TC::CLICmdReady &) {
    if (cmd_buf.is_empty()) {
      waiting_command_worker_tid = sender_tid;
      return;
    }
    reply(sender_tid, TC::CLICmd{cmd_buf.pop().value()});
  }

  template <class T> void handle(int sender_tid, const T &) {
    reply_with_error(sender_tid);
  }

  void run() {
    int sender_tid;
    Message msg;
    receive(&sender_tid, msg);
    std::visit([&](auto &&arg) { handle(sender_tid, arg); }, msg);
  };
};

void train_controller_program_task();