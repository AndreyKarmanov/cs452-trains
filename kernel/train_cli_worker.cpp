#include "train_cli_worker.h"
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

  bool parse_token(const char *&cursor, const char *end,
                   StaticString<8> &token) {
    cursor = skip_ws(cursor, end);
    if (cursor >= end)
      return false;

    token.clear();
    while (cursor < end && !isspace(static_cast<unsigned char>(*cursor))) {
      if (!token.append(*cursor))
        return false;
      ++cursor;
    }
    return !token.empty();
  }

} // namespace

TC::Cmd::Any parse_command(StaticString<CLI_BUFFER_SIZE> &buf) {
  TC::Cmd::Any out{};

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
    return TC::Cmd::Quit{};
  }

  if (cmd_len == 2 && strncmp(cmd, "tr", 2) == 0) {
    uint32_t loco_id      = 0;
    uint32_t speed        = 0;
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_uint(parse_cur, end, speed) && speed <= MAX_USER_SPEED &&
        done_parse(parse_cur, end)) {
      out = TC::Cmd::Speed{loco_id, speed};
      buf.set("Success: tr ", loco_id, ' ', speed);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is tr <train number> <speed 0-", MAX_USER_SPEED,
              '>');
    }
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "lr", 2) == 0) {
    uint32_t loco_id         = 0;
    uint32_t light           = 0;
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        parse_uint(parse_cursor, end, light) && done_parse(parse_cursor, end)) {
      out = TC::Cmd::Light{loco_id, light != 0};
      buf.set("Success: lr ", loco_id, ' ', light);
    } else {
      out = TC::Cmd::Invalid{};
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
      out = TC::Cmd::Switch{sw_id, dir == 'S' || dir == 's'};
      buf.set("Success: sw ", sw_id, ' ', dir);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is sw <switch number> <switch direction>");
    }
    return out;
  }

  // if (cmd_len == 2 && strncmp(cmd, "rv", 2) == 0) {
  //   uint32_t loco_id         = 0;
  //   const char *parse_cursor = cur;
  //   if (parse_uint(parse_cursor, end, loco_id) &&
  //       done_parse(parse_cursor, end)) {
  //     out = TC::Cmd::Reverse{loco_id, state.get_loco(loco_id).backward};
  //     buf.set("Success: rv ", loco_id, " (stopping)");
  //   } else {
  //     out = TC::Cmd::Invalid{};
  //     buf.set("Error: Format is rv <train number>");
  //   }
  //   return out;
  // }

  if (cmd_len == 4 && strncmp(cmd, "stop", 4) == 0 && done_parse(cur, end)) {
    out = TC::Cmd::Stop{};
    buf.set("Success: stop (stopping)");
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "go", 2) == 0 && done_parse(cur, end)) {
    out = TC::Cmd::Go{};
    buf.set("Success: go (starting)");
    return out;
  }

  if (cmd_len == 5 && strncmp(cmd, "reset", 5) == 0 && done_parse(cur, end)) {
    out = TC::Cmd::Reset{};
    buf.set("Success: reset (resetting all state)");
    return out;
  }

  if (cmd_len == 5 && strncmp(cmd, "quirk", 5) == 0 && done_parse(cur, end)) {
    out = TC::Cmd::RemoveTrains{};
    buf.set("Success: quirk (marlin quirk clear)");
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "rt", 2) == 0) {
    uint32_t loco_id      = 0;
    uint32_t value        = 0;
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_uint(parse_cur, end, value) && done_parse(parse_cur, end)) {
      out = TC::Cmd::RunTree{loco_id, value};
      buf.set("Success: rt ", loco_id, ' ', value);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is rt <train number> <value>");
    }
    return out;
  }

  if (cmd_len == 3 && strncmp(cmd, "nav", 3) == 0) {
    uint32_t loco_id = 0;
    StaticString<8> to{};
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_token(parse_cur, end, to)) {
      uint32_t speed = 7;
      parse_cur      = skip_ws(parse_cur, end);
      if (parse_cur < end &&
          (!parse_uint(parse_cur, end, speed) || speed > MAX_USER_SPEED ||
           !done_parse(parse_cur, end))) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Format is nav <train number> <to node> [<speed 0-",
                MAX_USER_SPEED, ">]");
        return out;
      }
      out = TC::Cmd::Nav{loco_id, to, speed};
      buf.set("Success: nav ", loco_id, ' ', to, ' ', speed);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is nav <train number> <to node> [<speed 0-",
              MAX_USER_SPEED, ">]");
    }
    return out;
  }

  out = TC::Cmd::Invalid{};
  buf.set("Error: cmds: q, tr, sw, rv, lr, stop, go, reset, quirk, rt, "
          "nav");
  return out;
}

void cli_worker() {
  auto rx_tid = WhoIs(UART_RX_Server::NAME);
  _assert(rx_tid >= 0, "SHELL: RX SERVER WHOIS FAILED");

  auto tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
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

      if (std::get_if<TC::Cmd::Invalid>(&result)) {
        continue;
      }

      auto cans_reply = send<TC::Ack>(tcs_tid, result);
      if (cans_reply.error()) {
        break;
      }

      if (std::get_if<TC::Cmd::Quit>(&result)) {
        break;
      }
    }
  }
  Offset_Puts(tx_tid, 0, "CLI EXITING\n\r");
}
