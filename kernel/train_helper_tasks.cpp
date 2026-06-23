#include "io_helpers.h"
#include "message.h"
#include "name_server.h"
#include "syscall.h"
#include "train_control.h"
#include "uart_tx_server.h"
#include <array>

void switch_medium_loop(int tcs_tid) {
  static constexpr std::array<std::pair<uint32_t, bool>, 8> switches{{
      {14, false}, // c
      {13, true},  // s
      {10, true},  // s
      {9, false},  // c
      {8, false},  // c
      {17, true},  // s
      {16, true},  // s
      {15, false}, // c
  }};

  for (const auto &[sw_id, straight] : switches) {
    auto reply =
        send<TC::Ack>(tcs_tid, TC::CLICmd{UserCmd::Switch{sw_id, straight}});
    _assert(reply.has_value(), "SWITCH MEDIUM LOOP FAILED");
  }
}

void cal_speed_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto params = send<TC::CalSpeedParams>(tcs_tid, TC::CalSpeedReady{});
  _assert(params.has_value(), "CAL SPEED PARAMS FAILED");

  switch_medium_loop(tcs_tid);

  auto speed_reply = send<TC::Ack>(
      tcs_tid, TC::CLICmd{UserCmd::Speed{params->loco_id, params->speed}});
  _assert(speed_reply.has_value(), "CAL SPEED SET SPEED FAILED");

  for (int i = 0; i < 3; ++i) {
    await_event(Event::SENSOR_B6);
    Puts(tx_tid, "calspeed: sensor B6 trigger ", i + 1, "/3\n\r");
  }

  auto stop_reply =
      send<TC::Ack>(tcs_tid, TC::CLICmd{UserCmd::Speed{params->loco_id, 0}});
  _assert(stop_reply.has_value(), "CAL SPEED STOP FAILED");

  Debug_Puts(tx_tid, "test done\n\r");
}
