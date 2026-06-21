#pragma once

#include "debug.h"
#include "io_helpers.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"
#include "train_state.h"
#include "tx_server.h"
#include <array>
#include <cstddef>
#include <ctype.h>

constexpr size_t USER_CMD_TIMING_COUNT = 9;

uint32_t print_state(int tx_tid, const State &state,
                     const std::array<uint32_t, USER_CMD_TIMING_COUNT> &timings,
                     bool timings_dirty);

template <size_t CLI_BUFFER_SIZE = 64> class TrainUIServer {
  struct PendingTiming {
    UserCmd cmd;
    uint32_t start_time;
  };

  StaticString<CLI_BUFFER_SIZE> buf{};
  Buffer<UserCmd, 8> cmd_buf;
  std::array<PendingTiming, USER_CMD_TIMING_COUNT> pending_timings{};

  State state{};
  std::array<uint32_t, USER_CMD_TIMING_COUNT> command_timings{};
  bool timings_dirty = true;

  int tx_tid;

  static void ui_update_worker();
  static void cli_worker();
  static void command_worker();
  int waiting_command_worker_tid       = -1;
  uint32_t waiting_command_worker_time = 0;

  UserCmd parse_command();
  void enqueue_command_timing(const UserCmd &command, uint32_t start_ticks) {
    if (user_command_applied(state, command)) {
      return;
    }

    auto &pending      = pending_timings[static_cast<size_t>(command.type)];
    pending.cmd        = command;
    pending.start_time = start_ticks;
  }

  void update_pending_timings(uint32_t now) {
    for (auto &pending : pending_timings) {
      if (pending.start_time == 0) {
        continue;
      }

      if (!user_command_applied(state, pending.cmd)) {
        continue;
      }

      auto cmd_index             = static_cast<size_t>(pending.cmd.type);
      command_timings[cmd_index] = now - pending.start_time;
      pending.start_time         = 0;
      timings_dirty              = true;
    }
  }

  static bool user_command_applied(const State &current,
                                   const UserCmd &command) {
    switch (command.type) {
    case UserCmd::Type::Light: {
      for (const Train &train : current.trains) {
        if (train.loco_id == command.id) {
          return train.light_on == command.flag;
        }
      }
      return false;
    }
    case UserCmd::Type::Speed: {
      for (const Train &train : current.trains) {
        if (train.loco_id == command.id) {
          return train.requested_speed == static_cast<uint16_t>(command.value);
        }
      }
      return false;
    }
    case UserCmd::Type::Switch:
      return State::is_switch_id(static_cast<uint16_t>(command.id)) &&
             current.is_switch_straight(static_cast<uint16_t>(command.id)) ==
                 command.flag;
    case UserCmd::Type::Reverse: {
      const Train current_train = current.get_loco(command.id);
      return current_train.backward != command.flag;
    }
    case UserCmd::Type::Stop:
      return current.stopped;
    case UserCmd::Type::Go:
      return !current.stopped;
    case UserCmd::Type::Reset: {
      State default_state{};
      if (current.stopped != default_state.stopped ||
          current.switches != default_state.switches) {
        return false;
      }

      for (size_t i = 0; i < MAX_TRAINS; ++i) {
        const Train &lhs_train = current.trains[i];
        const Train &rhs_train = default_state.trains[i];
        if (lhs_train.loco_id != rhs_train.loco_id ||
            lhs_train.requested_speed != rhs_train.requested_speed ||
            lhs_train.backward != rhs_train.backward ||
            lhs_train.light_on != rhs_train.light_on) {
          return false;
        }
      }

      return true;
    }
    case UserCmd::Type::Invalid:
    case UserCmd::Type::Quit:
    default:
      return false;
    }
  }

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
      enqueue_command_timing(result, msg.time);
      if (waiting_command_worker_tid >= 0 &&
          result.type != UserCmd::Type::Invalid) {
        reply(waiting_command_worker_tid, TC::CLICmd{result});
        waiting_command_worker_tid = -1;
      } else if (result.type != UserCmd::Type::Invalid) {
        _assert(cmd_buf.push(result), "COMMAND BUFFER FULL");
      }
    }

    reply(sender_tid, TC::Ack{});
  }

  void handle(int sender_tid, const TC::UIUpdate &msg) {
    state = msg.state;
    update_pending_timings(msg.time);
    print_state(tx_tid, state, command_timings, timings_dirty);
    timings_dirty = false;
    reply(sender_tid, TC::Ack{});
  }

  void handle(int sender_tid, const TC::CLICmdReady &) {
    if (cmd_buf.is_empty()) {
      waiting_command_worker_tid = sender_tid;
      return;
    }
    auto cmd = cmd_buf.pop().value();
    reply(sender_tid, TC::CLICmd{cmd});
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