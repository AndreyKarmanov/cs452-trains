#include "train_ui_server.h"
#include "can_rx_server.h"
#include "can_server.h"
#include "clock_server.h"
#include "mrk.h"
#include "time.h"
#include "train_control.h"
#include "uart_rx_server.h"

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

template <> UserCmd::Cmd TrainUIServer<>::parse_command() {
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

  if (cmd_len == 2 && strncmp(cmd, "rv", 2) == 0) {
    uint32_t loco_id         = 0;
    const char *parse_cursor = cur;
    if (parse_uint(parse_cursor, end, loco_id) &&
        done_parse(parse_cursor, end)) {
      out = UserCmd::Reverse{loco_id, state.get_loco(loco_id).backward};
      buf.set("Success: rv ", loco_id, " (stopping)");
    } else {
      out = UserCmd::Invalid{};
      buf.set("Error: Format is rv <train number>");
    }
    return out;
  }

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

  out = UserCmd::Invalid{};
  buf.set("Error: cmds: q, tr, sw, rv, lr, stop, go, reset, quirk, rt");
  return out;
}

static constexpr int STATE_ROW_INT = 8;
static constexpr int STATUS_ROW    = STATE_ROW_INT + 1;
static constexpr int TRAIN_ROW     = STATE_ROW_INT + 3;
static constexpr int SENSOR_ROW    = TRAIN_ROW + MAX_TRAINS + 2;
static constexpr int SWITCH_ROW    = SENSOR_ROW + 3;

uint32_t print_state(int tx_tid, const State &state) {
  uint32_t draws = 0;
  StaticString<512> line;

  if (state.status_dirty) {
    line.set("\033[", STATUS_ROW, ";2HTrack ",
             state.stopped ? "Stopped" : "Active", "  \n\r");
    Puts(tx_tid, line);
    ++draws;
  }

  if (state.trains_dirty) {
    line.set("\033[", TRAIN_ROW, ";2HTrain | Dir | Lamp | Speed \n\r");
    for (const TrainState &train : state.trains) {
      line.append("\033[K   ", train.loco_id, "  | ",
                  train.backward ? "Rev" : "Fwd", " | ",
                  train.light_on ? " On " : " Off", " | ",
                  train.requested_speed, "\n\r");
    }
    Puts(tx_tid, line);
    ++draws;
  }

  if (state.sensors_dirty) {
    line.set("\033[", SENSOR_ROW, ";2HRecent Sensors \n\r\033[K   ");
    for (size_t i = state.sensors.size(); i-- > 0;) {
      uint16_t s_id = state.sensors[i].value();
      char bank     = 'A' + ((s_id - 1) / 16);
      int number    = ((s_id - 1) % 16) + 1;
      line.append(bank, number, ' ');
    }
    line.append("\n\r");
    Puts(tx_tid, line);
    ++draws;
  }

  if (state.switches_dirty) {
    line.set("\033[", SWITCH_ROW, ";2HSwitches\n\r");
    for (int sw_id = 0; sw_id < 22; ++sw_id) {
      const char c =
          state.is_switch_straight(State::switch_id(sw_id)) ? 'S' : 'C';

      if (sw_id < 9) {
        line.append("   ", sw_id + 1, "  : ", c);
      } else if (sw_id < 18) {
        line.append("   ", sw_id + 1, " : ", c);
      } else {
        line.append("   ", sw_id + 135, ": ", c);
      }
      if (sw_id % 4 == 3) {
        line.append("\n\r");
      }
    }
    Puts(tx_tid, line);
    ++draws;
  }

  return draws;
}

template <> void TrainUIServer<>::ui_update_worker() {
  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  auto uis_tid = WhoIs(TrainUIServer<>::TC_UI_SERVER_NAME);
  _assert(uis_tid >= 0, "TC SERVER NOT FOUND");

  while (true) {
    auto cans_reply = send<TC::UIUpdate>(tcs_tid, TC::UIReady{});
    if (!cans_reply.has_value()) {
      break;
    }
    auto uis_reply = send<TC::Ack>(
        uis_tid, TC::UIUpdate{.state = cans_reply->state,
                              .time  = static_cast<uint32_t>(Time(cs_tid))});
    if (!uis_reply.has_value()) {
      break;
    }
  }
}

template <> void TrainUIServer<>::ui_print_worker() {
  auto tcs_tid = WhoIs(TrainUIServer<>::TC_UI_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  while (true) {
    auto print = send<TC::UIPrint>(tcs_tid, TC::UIPrintReady{});
    if (!print.has_value()) {
      break;
    }
    print_state(tx_tid, print->state);
  }
}

template <> void TrainUIServer<>::cli_worker() {
  auto rx_tid = WhoIs(UART_RX_Server::RX_SERVER_NAME);
  _assert(rx_tid >= 0, "SHELL: RX SERVER WHOIS FAILED");

  auto uis_tid = WhoIs(TrainUIServer<>::TC_UI_SERVER_NAME);
  _assert(uis_tid >= 0, "TC SERVER NOT FOUND");

  auto cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  TC::CLIInput msg{};
  while (true) {
    msg.c    = Getc(rx_tid);
    msg.time = static_cast<uint32_t>(Time(cs_tid));

    auto uis_reply = send<TC::Ack>(uis_tid, msg);
    if (!uis_reply.has_value()) {
      break;
    }
  }
}

template <> void TrainUIServer<>::command_worker() {
  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto uis_tid = WhoIs(TrainUIServer<>::TC_UI_SERVER_NAME);
  _assert(uis_tid >= 0, "TC SERVER NOT FOUND");

  TC::CLICmdReady msg{};
  while (true) {
    auto uis_reply = send<TC::CLICmd>(uis_tid, msg);
    if (!uis_reply.has_value()) {
      break;
    }

    auto cans_reply = send<TC::Ack>(tcs_tid, uis_reply.value());
    if (!cans_reply.has_value()) {
      break;
    }
  }
}

static void train_ui_server_task() {
  TrainUIServer<> server;
  for (;;) {
    server.run();
  }
}

static void train_control_server_task() {
  TrainControlServer<> server;
  for (;;) {
    server.run();
  }
}

void train_controller_program_task() {
  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");
  Puts(tx_tid, "\033[2J\033[1;1H");

  create(2, can_rx_server_task);
  create(2, can_server_task);
  create(2, train_control_server_task);
  create(3, train_ui_server_task);
}
