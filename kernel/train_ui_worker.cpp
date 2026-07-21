#include "io_helpers.h"
#include "message.h"
#include "time.h"
#include "train_control.h"
#include "uart_tx_server.h"

static constexpr int STATE_ROW_INT = 8;
static constexpr int STATUS_ROW    = STATE_ROW_INT + 1;
static constexpr int TRAIN_ROW     = STATE_ROW_INT + 3;
static constexpr int SENSOR_ROW    = TRAIN_ROW + MAX_TRAINS * 3 + 2;
static constexpr int SWITCH_ROW    = SENSOR_ROW + 3;

void print_state(int tx_tid, int web_tid, const State &state, State &prev) {
  StaticString<TX::MAX_DATA_LENGTH> line;
  static Track track(TrainControlServer<>::TRACK);

  if (state.stopped != prev.stopped) {
    line.set("\033[", STATUS_ROW, ";2HTrack ",
             state.stopped ? "Stopped" : "Active", "  \n\r");
    Puts(tx_tid, line);
  }

  // any protocol you want, here it is!
  if (web_tid >= 0 && state.reservations != prev.reservations) {
    StaticString<128> res_print{};
    for (const auto &[node, value] : state.reservations) {
      res_print.append(track[node.node_idx].name,
                       track[node.node_idx].type == NODE_BRANCH
                           ? (node.edge_dir ? "C" : "S")
                           : "",
                       ",");
    }
    WebSerial_Puts(web_tid, "Reserved nodes: ");
    WebSerial_Puts(web_tid, res_print.c_str());
    WebSerial_Puts(web_tid, "\n\r");
  }

  if (state.trains != prev.trains) {
    line.set("\033[", TRAIN_ROW,
             ";2HTr | D | L | spd | est | stop | last | dx\n\r");
    for (const TrainState &train : state.trains) {

      if (train == *prev.get_loco(train.id)) {
        // skip lines if the train state hasn't changed
        Puts(tx_tid, "\n\r\n\r\n\r");
        continue;
      }
      line.append(" ", train.id, " | ", train.backward ? "R" : "F", " | ",
                  train.light_on ? "1" : "0", " | ");
      AppendPadded(line, train.req_speed, 3);
      line.append(" | ");
      AppendPadded(line, train.ve_nm / 1000, 3);
      line.append(" | ");
      AppendPadded(line, train.stop_dist_um / 1000, 4);
      line.append(" |  ");
      if (train.last_sensor.has_value()) {
        AppendPadded(line, train.last_sensor->sens.sid, 3);
      } else {
        line.append("---");
      }
      line.append(" | ", train.d_um / 1000);

      auto train_path = train.e_path.decode(track);
      line.append("\033[K\n\r", train_path.to_string(&track), "\033[K\n\r");

      for (const auto &[key, value] : state.reservations) {
        if (value != train.id) {
          continue;
        }
        line.append(track[key.node_idx].name, key.edge_dir ? "C" : "S", " ");
      }
      line.append("\033[K\n\r");
      Puts(tx_tid, line);
    }
  }

  if (state.sensors != prev.sensors) {
    line.set("\033[", SENSOR_ROW, ";2HRecent Sensors \n\r\033[K   ");
    for (size_t i = state.sensors.size(); i-- > 0;) {
      uint16_t s_id = state.sensors[i].value();
      char bank     = 'A' + ((s_id - 1) / 16);
      int number    = ((s_id - 1) % 16) + 1;
      line.append(bank, number, ' ');
    }
    line.append("\n\r");
    Puts(tx_tid, line);
  }

  if (state.switches != prev.switches) {
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
  }
}
void ui_update_worker() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto cs_tid = WhoIs(ClockServer<>::NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  auto web_tid = WhoIs(UART03_TX_Server::NAME);
  _assert(web_tid >= 0, "TX SERVER3 WHOIS FAILED");

  State prev_state{};
  while (true) {
    auto cans_reply = send<TC::UIUpdate>(tcs_tid, TC::UIReady{});
    if (!cans_reply.has_value()) {
      break;
    }
#if !defined(DATA_COLLECTION) || !DATA_COLLECTION
    print_state(tx_tid, web_tid, cans_reply->state, prev_state);
#endif
    prev_state = cans_reply->state;
    Delay(cs_tid, TICKS_PER_S / 10);
  }

  Debug_Puts(tx_tid, "UI EXITING\n\r");
}