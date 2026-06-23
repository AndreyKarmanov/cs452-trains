#include "train_cli_worker.h"
#include "clock_server.h"
#include "io_helpers.h"
#include "mrk.h"
#include "time.h"
#include "train_control.h"
#include "uart_rx_server.h"
#include <ctype.h>
#include <variant>

namespace {

  const char *skip_ws(const char *p, const char *end) {
    while (p < end && isspace(static_cast<unsigned char>(*p)))
      ++p;
    return p;
  }

  bool parse_uint(const char *&cursor, const char *end, uint32_t &value) {
    cursor = skip_ws(cursor, end);
    if (cursor >= end || !isdigit(static_cast<unsigned char>(*cursor)))
      return false;

    uint32_t result = 0;
    while (cursor < end && isdigit(static_cast<unsigned char>(*cursor))) {
      result = result * 10 + static_cast<uint32_t>(*cursor - '0');
      ++cursor;
    }

    if (cursor < end && !isspace(static_cast<unsigned char>(*cursor)))
      return false;

    value = result;
    return true;
  }

  bool done_parse(const char *cursor, const char *end) {
    return skip_ws(cursor, end) == end;
  }

} // namespace

UserCmd::Cmd parse_command(StaticString<CLI_BUFFER_SIZE> &buf) {
  UserCmd::Cmd out{};

  const char *begin = buf.data;
  const char *end   = buf.data + buf.len;
  const char *cmd   = skip_ws(begin, end);
  if (cmd == end) {
    return out;
  }

  const char *cur = cmd;
  while (cur < end && !isspace(static_cast<unsigned char>(*cur)))
    ++cur;
  size_t cmd_len = static_cast<size_t>(cur - cmd);

  if (cmd_len == 1 && (cmd[0] == 'q' || cmd[0] == 'Q') &&
      done_parse(cur, end)) {
    buf.set("Success: q (quit)");
    return UserCmd::Quit{};
  }

  if (cmd_len == 2 && strncmp(cmd, "tr", 2) == 0) {
    uint32_t loco_id      = 0;
    uint32_t speed        = 0;
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_uint(parse_cur, end, speed) && done_parse(parse_cur, end)) {
      out = UserCmd::Speed{loco_id, speed};
      buf.set("Success: tr ", loco_id, ' ', speed);
    } else {
      out = UserCmd::Invalid{};
      buf.set("Error: Format is tr <train number> <train speed>");
    }
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "lr", 2) == 0) {
    uint32_t loco_id         = 0;
    uint32_t light           = 0;
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        parse_uint(parse_cursor, end, light) && done_parse(parse_cursor, end)) {
      out = UserCmd::Light{loco_id, light != 0};
      buf.set("Success: lr ", loco_id, ' ', light);
    } else {
      out = UserCmd::Invalid{};
      buf.set("Error: Format is lr <train number> <light state>");
    }
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "sw", 2) == 0) {
    uint32_t sw_id           = 0;
    char dir                 = '\0';
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, sw_id)) {
      parse_cursor = skip_ws(parse_cursor, end);
      if (parse_cursor < end) {
        dir = *parse_cursor++;
      }
    }
    if (sw_id != 0 && (dir == 'S' || dir == 's' || dir == 'C' || dir == 'c') &&
        done_parse(parse_cursor, end)) {
      out = UserCmd::Switch{sw_id, dir == 'S' || dir == 's'};
      buf.set("Success: sw ", sw_id, ' ', dir);
    } else {
      out = UserCmd::Invalid{};
      buf.set("Error: Format is sw <switch number> <switch direction>");
    }
    return out;
  }

  // if (cmd_len == 2 && strncmp(cmd, "rv", 2) == 0) {
  //   uint32_t loco_id         = 0;
  //   const char *parse_cursor = cur;
  //   if (parse_uint(parse_cursor, end, loco_id) &&
  //       done_parse(parse_cursor, end)) {
  //     out = UserCmd::Reverse{loco_id, state.get_loco(loco_id).backward};
  //     buf.set("Success: rv ", loco_id, " (stopping)");
  //   } else {
  //     out = UserCmd::Invalid{};
  //     buf.set("Error: Format is rv <train number>");
  //   }
  //   return out;
  // }

  if (cmd_len == 4 && strncmp(cmd, "stop", 4) == 0 && done_parse(cur, end)) {
    out = UserCmd::Stop{};
    buf.set("Success: stop (stopping)");
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "go", 2) == 0 && done_parse(cur, end)) {
    out = UserCmd::Go{};
    buf.set("Success: go (starting)");
    return out;
  }

  if (cmd_len == 5 && strncmp(cmd, "reset", 5) == 0 && done_parse(cur, end)) {
    out = UserCmd::Reset{};
    buf.set("Success: reset (resetting all state)");
    return out;
  }

  if (cmd_len == 5 && strncmp(cmd, "quirk", 5) == 0 && done_parse(cur, end)) {
    out = UserCmd::RemoveTrains{};
    buf.set("Success: quirk (marlin quirk clear)");
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "rt", 2) == 0) {
    uint32_t loco_id      = 0;
    uint32_t value        = 0;
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_uint(parse_cur, end, value) && done_parse(parse_cur, end)) {
      out = UserCmd::RunTree{loco_id, value};
      buf.set("Success: rt ", loco_id, ' ', value);
    } else {
      out = UserCmd::Invalid{};
      buf.set("Error: Format is rt <train number> <value>");
    }
    return out;
  }

  if (cmd_len == 8 && strncmp(cmd, "calspeed", 8) == 0) {
    uint32_t loco_id      = 0;
    uint32_t speed        = 0;
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_uint(parse_cur, end, speed) && done_parse(parse_cur, end)) {
      out = UserCmd::CalSpeed{loco_id, speed};
      buf.set("Success: calspeed ", loco_id, ' ', speed);
    } else {
      out = UserCmd::Invalid{};
      buf.set("Error: Format is calspeed <train number> <train speed>");
    }
    return out;
  }

  out = UserCmd::Invalid{};
  buf.set(
      "Error: cmds: q, tr, sw, rv, lr, stop, go, reset, quirk, rt, calspeed");
  return out;
}

void cli_worker() {
  auto rx_tid = WhoIs(UART_RX_Server::RX_SERVER_NAME);
  _assert(rx_tid >= 0, "SHELL: RX SERVER WHOIS FAILED");

  auto cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  StaticString<CLI_BUFFER_SIZE> buf{};

  while (true) {
    char c = Getc(rx_tid);
    if (isprint(c)) {
      if (!buf.append(c)) {
        continue;
      }
      Puts(tx_tid, "\033[", CONSOLE_LINE, ";1H\033[K> ", buf, "\n\r");
    } else if ((c == 0x08 || c == 0x7f) && !buf.empty()) { // backspace
      buf.pop_back();
      Puts(tx_tid, "\033[", CONSOLE_LINE, ";1H\033[K> ", buf, "\n\r");
    } else if (c == '\r') { // enter
      auto result = parse_command(buf);
      Puts(tx_tid, "\033[", CONSOLE_LINE, ";1H\033[K> ", buf, "\n\r");
      buf.clear();

      if (std::get_if<UserCmd::Invalid>(&result)) {
        continue;
      }

      auto cans_reply = send<TC::Ack>(tcs_tid, TC::CLICmd{result});
      if (!cans_reply.has_value()) {
        break;
      }
    }
  }
  Debug_Puts(tx_tid, "CLI WORKER EXITING\n\r");
}
