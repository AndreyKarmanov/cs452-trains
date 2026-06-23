#include "io_helpers.h"
#include "message.h"
#include "name_server.h"
#include "syscall.h"
#include "train_control.h"
#include "uart_tx_server.h"
#include <array>
#include <cstdint>

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

void cal_speed_task_at_speed(uint32_t loco_id, uint32_t speed, int tcs_tid,
                             int tx_tid) {
  Debug_Puts(tx_tid, "calspeed: setting speed ", speed, " for train ", loco_id);
  auto speed_reply =
      send<TC::Ack>(tcs_tid, TC::CLICmd{UserCmd::Speed{loco_id, speed}});
  _assert(speed_reply.has_value(), "CAL SPEED SET SPEED FAILED");

  // run laps
  for (int i = 0; i < 1; ++i) {
    await_event(Event::SENSOR_B6);
  }
}

void cal_speed_task() {
  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  // debug sensor on to track times and sensor triggers
  auto debug_on_reply =
      send<TC::Ack>(tcs_tid, TC::CLICmd{UserCmd::DebugSensor{true}});
  _assert(debug_on_reply.has_value(), "CAL SPEED DEBUG SENSOR ON FAILED");

  auto params = send<TC::CalSpeedParams>(tcs_tid, TC::CalSpeedReady{});
  _assert(params.has_value(), "CAL SPEED PARAMS FAILED");

  switch_medium_loop(tcs_tid);

  // if speed == 0, then we will do all speed levels from 1-14
  if (params->speed == 0) {
    for (uint32_t i = 7; i <= 14; ++i) {
      cal_speed_task_at_speed(params->loco_id, i, tcs_tid, tx_tid);
    }
  } else { // otherwise, just do the one speed
    cal_speed_task_at_speed(params->loco_id, params->speed, tcs_tid, tx_tid);
  }

  // stop train
  auto stop_reply =
      send<TC::Ack>(tcs_tid, TC::CLICmd{UserCmd::Speed{params->loco_id, 0}});
  _assert(stop_reply.has_value(), "CAL SPEED STOP FAILED");

  // debug off
  auto debug_off_reply =
      send<TC::Ack>(tcs_tid, TC::CLICmd{UserCmd::DebugSensor{false}});
  _assert(debug_off_reply.has_value(), "CAL SPEED DEBUG SENSOR OFF FAILED");
}
