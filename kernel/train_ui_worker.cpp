#include "io_helpers.h"
#include "message.h"
#include "time.h"
#include "train_control.h"
#include "uart_tx_server.h"

static constexpr int STATE_ROW_INT = 8;
static constexpr int STATUS_ROW    = STATE_ROW_INT + 1;
static constexpr int TRAIN_ROW     = STATE_ROW_INT + 3;
static constexpr int SENSOR_ROW    = TRAIN_ROW + MAX_TRAINS * 2 + 2;
static constexpr int SWITCH_ROW    = SENSOR_ROW + 3;

uint32_t print_state(int tx_tid, const State &state) {
  uint32_t draws = 0;
  StaticString<TX::MAX_DATA_LENGTH> line;
  static Track track(TrainControlServer<>::TRACK);

  if (state.status_dirty) {
    line.set("\033[", STATUS_ROW, ";2HTrack ",
             state.stopped ? "Stopped" : "Active", "  \n\r");
    Puts(tx_tid, line);
    ++draws;
  }

  if (state.trains_dirty) {
    line.set("\033[", TRAIN_ROW,
             ";2HTr | D | L | spd | est | tgt | stop | last | dx\n\r");
    for (const TrainState &train : state.trains) {
      line.append(" ", train.id, " | ", train.backward ? "R" : "F", " | ",
                  train.light_on ? "1" : "0", " | ");
      AppendPadded(line, train.req_speed, 3);
      line.append(" | ");
      AppendPadded(line, train.ve_nm / 1000, 3);
      line.append(" | ");
      AppendPadded(line, train.target_node_idx, 3);
      line.append(" | ");
      AppendPadded(line, train.stop_dist_um / 1000, 4);
      line.append(" |  ");
      line.append(train.last_sensor.has_value()
                      ? train.last_sensor->sens.to_string()
                      : "---");
      line.append(" | ");
      AppendPadded(line, train.d_um / 1000, 3);
      line.append("\n\r", train.e_path.decode(track).to_string(&track),
                  "\033[K\n\r");
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
void ui_update_worker() {
  auto tcs_tid = WhoIs(TrainControlServer<>::NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto tx_tid = WhoIs(UART_TX_Server::NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto cs_tid = WhoIs(ClockServer<>::NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  while (true) {
    auto cans_reply = send<TC::UIUpdate>(tcs_tid, TC::UIReady{});
    if (!cans_reply.has_value()) {
      break;
    }
#if !defined(DATA_COLLECTION) || !DATA_COLLECTION
    print_state(tx_tid, cans_reply->state);
#endif
    Delay(cs_tid, TICKS_PER_S / 10);
  }

  Offset_Puts(tx_tid, 1, "UI EXITING\n\r");
}