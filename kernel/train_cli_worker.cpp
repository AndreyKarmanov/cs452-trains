#include "train_cli_worker.h"
#include "io_helpers.h"
#include "mrk.h"
#include "pathfind.h"
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

  bool parse_int(const char *&cursor, const char *end, int &value) {
    cursor = skip_ws(cursor, end);
    if (cursor >= end)
      return false;

    bool negative = false;
    if (*cursor == '-') {
      negative = true;
      ++cursor;
    }

    uint32_t result = 0;
    if (!parse_uint(cursor, end, result))
      return false;

    value = negative ? -static_cast<int>(result) : static_cast<int>(result);
    return true;
  }

  bool done_parse(const char *cursor, const char *end) {
    return skip_ws(cursor, end) == end;
  }

  template <size_t SIZE>
  bool parse_token(const char *&cursor, const char *end,
                   StaticString<SIZE> &token) {
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

  bool parse_edge_dir(const char *&cursor, const char *end, int &edge_dir,
                      char &dir_label) {
    StaticString<2> token{};
    if (!parse_token(cursor, end, token) || token.len != 1)
      return false;

    const char dir = token.data[0];
    if (dir == 'S' || dir == 's') {
      edge_dir  = 0;
      dir_label = 'S';
      return true;
    }

    if (dir == 'C' || dir == 'c') {
      edge_dir  = 1;
      dir_label = 'C';
      return true;
    }

    return false;
  }

} // namespace

TC::Cmd::Any parse_command(StaticString<CLI_BUFFER_SIZE> &buf) {
  TC::Cmd::Any out{};
  static Track track{TrainControlServer<>::TRACK};

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

  if (cmd_len == 2 && strncmp(cmd, "lf", 2) == 0) {
    uint32_t loco_id         = 0;
    uint32_t function        = 0;
    uint32_t value           = 0;
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        parse_uint(parse_cursor, end, function) &&
        parse_uint(parse_cursor, end, value) && done_parse(parse_cursor, end)) {
      out = TC::Cmd::Function{loco_id, static_cast<uint8_t>(function),
                              static_cast<uint8_t>(value)};
      buf.set("Success: lf ", loco_id, ' ', function, ' ', value);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is lf <train number> <function> <state>");
    }
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "sw", 2) == 0) {
    uint32_t sw_id           = 0;
    int edge_dir             = 0;
    char dir_label           = '\0';
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, sw_id) &&
        parse_edge_dir(parse_cursor, end, edge_dir, dir_label) &&
        done_parse(parse_cursor, end)) {
      out = TC::Cmd::Switch{sw_id, edge_dir == 0};
      buf.set("Success: sw ", sw_id, ' ', dir_label);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is sw <switch number> <S/C>");
    }
    return out;
  }

  if (cmd_len == 2 && strncmp(cmd, "rv", 2) == 0) {
    uint32_t loco_id         = 0;
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        done_parse(parse_cursor, end)) {
      out = TC::Cmd::Reverse{loco_id};
      buf.set("Success: rv ", loco_id, " (stopping)");
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is rv <train number>");
    }
    return out;
  }

  if (cmd_len == 3 && strncmp(cmd, "res", 3) == 0) {
    uint32_t loco_id = 0;
    Track::NodeName node_name{};
    int edge_dir   = 0;
    char dir_label = '\0';

    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        parse_token(parse_cursor, end, node_name) &&
        parse_edge_dir(parse_cursor, end, edge_dir, dir_label) &&
        done_parse(parse_cursor, end)) {
      auto node_idx = track.get_idx(node_name);
      if (!node_idx.has_value()) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Unknown node name in res command");
        return out;
      }
      out = TC::Cmd::Reserve{loco_id, node_idx.value(), edge_dir};
      buf.set("Success: res ", loco_id, " ", node_name, " ", dir_label);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is res <train number> <node name> <S/C>");
    }
    return out;
  }

  if (cmd_len == 3 && strncmp(cmd, "rel", 3) == 0) {
    uint32_t loco_id = 0;
    Track::NodeName node_name{};
    int edge_dir   = 0;
    char dir_label = '\0';

    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        parse_token(parse_cursor, end, node_name) &&
        parse_edge_dir(parse_cursor, end, edge_dir, dir_label) &&
        done_parse(parse_cursor, end)) {
      auto node_idx = track.get_idx(node_name);
      if (!node_idx.has_value()) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Unknown node name in rel command");
        return out;
      }
      out = TC::Cmd::ReleaseReserve{loco_id, node_idx.value(), edge_dir};
      buf.set("Success: rel ", loco_id, " ", node_name, " ", dir_label);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is rel <train number> <node name> <S/C>");
    }
    return out;
  }

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
    uint32_t loco_id   = 0;
    uint32_t tree_type = 0;
    int value1         = 0;
    int value2         = 0;
    int value3         = 0;

    const char *parse_cur = cur;

    // try to parse the required ones
    if (!(parse_uint(parse_cur, end, loco_id) &&
          parse_uint(parse_cur, end, tree_type))) {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is rt <train number> <tree> [<value1> <value2> "
              "<value3>]");
      return out;
    }

    // if required succeeded, try parsing the optional ones
    parse_int(parse_cur, end, value1);
    parse_int(parse_cur, end, value2);
    parse_int(parse_cur, end, value3);

    // Check if we are done parsing. Above parse will leave tokens if they
    // failed
    if (!done_parse(parse_cur, end)) {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is rt <train number> <tree> [<value> <value2> "
              "<value3>]");
    }
    out = TC::Cmd::RunTree{.id        = loco_id,
                           .tree_type = static_cast<TC::Tree::Type>(tree_type),
                           .value1    = value1,
                           .value2    = value2,
                           .value3    = value3};

    buf.set("Success: rt ", loco_id, ' ', tree_type, ' ', value1, ' ', value2,
            ' ', value3);

    return out;
  }

  if (cmd_len == 3 && strncmp(cmd, "reg", 3) == 0) {
    uint32_t loco_id = 0;
    StaticString<8> sensor{};
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_token(parse_cur, end, sensor) && done_parse(parse_cur, end)) {
      auto node_idx = track.get_idx(sensor);
      if (!node_idx.has_value()) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Unknown node name in reg command");
        return out;
      }

      if (track[sensor].type != NODE_SENSOR) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Node is not a sensor in reg command");
        return out;
      }
      out = TC::Cmd::Reg{loco_id, node_idx.value()};
      buf.set("Success: reg ", loco_id, ' ', sensor);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is reg <train number> <node name>");
    }
    return out;
  }

  if (cmd_len == 3 && strncmp(cmd, "nav", 3) == 0) {
    uint32_t loco_id = 0;
    StaticString<8> to{};
    uint32_t speed        = 7;
    const char *parse_cur = cur;
    if (parse_uint(parse_cur, end, loco_id) &&
        parse_token(parse_cur, end, to) && parse_uint(parse_cur, end, speed) &&
        speed <= MAX_USER_SPEED && speed > 0) {
      parse_cur  = skip_ws(parse_cur, end);
      int offset = 0;

      if (parse_cur < end &&
          (!parse_int(parse_cur, end, offset) || !done_parse(parse_cur, end))) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Format is nav <train number> <to node> <speed 0-",
                MAX_USER_SPEED, "> [<offset>]");
        return out;
      }
      auto node_idx = track.get_idx(to);
      if (!node_idx.has_value()) {
        out = TC::Cmd::Invalid{};
        buf.set("Error: Unknown node name in nav command");
        return out;
      }
      out = TC::Cmd::Nav{.id       = loco_id,
                         .node_idx = node_idx.value(),
                         .speed    = speed,
                         .offset   = offset};
      buf.set("Success: nav ", loco_id, ' ', to, ' ', speed, ' ', offset);
    } else {
      out = TC::Cmd::Invalid{};
      buf.set("Error: Format is nav <train number> <to node> <speed 0-",
              MAX_USER_SPEED, "> [<offset>]");
    }
    return out;
  }

  out = TC::Cmd::Invalid{};
  buf.set("Error: cmds: q, tr, sw, rv, lr, stop, go, reset, quirk, rt, "
          "reg, nav");
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
      if (!cans_reply.has_value()) {
        break;
      }

      if (auto data = std::get_if<TC::Cmd::Quit>(&result); data != nullptr) {
        break;
      }
    }
  }
  Debug_Puts(tx_tid, "CLI EXITING\n\r");
}
