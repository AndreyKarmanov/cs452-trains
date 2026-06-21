#include "train_control.h"
#include "can_server.h"

void train_control_can_courier_task() {
  auto tc_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tc_tid >= 0, "TC SERVER WHOIS FAILED");

  auto can_tid = WhoIs(CanServer<>::CAN_SERVER_NAME);
  _assert(can_tid >= 0, "CAN SERVER WHOIS FAILED");

  while (true) {
    auto tx_msg = send<TC::TX>(tc_tid, TC::TXReady{});
    if (!tx_msg.has_value()) {
      break;
    }

    auto frame      = encode_frame(tx_msg->mrk);
    int send_result = tx_msg->delay_ticks > 0
                          ? CanDelay(can_tid, frame, tx_msg->delay_ticks)
                          : CanSend(can_tid, frame);
    _assert(send_result == 0, "CAN COURRIER SEND FAILED");
  }
}

size_t expand_user_command(const State &state, const UserCmd &command,
                           std::array<MRKCmd, 64> &commands) {
  size_t command_count = 0;
  auto push            = [&](const auto &cmd) {
    if (command_count < commands.size()) {
      commands[command_count++] = cmd;
    }
  };

  switch (command.type) {
  case UserCmd::Type::Light:
    push(LightCmd(command.id, command.flag));
    break;
  case UserCmd::Type::Speed:
    push(SpeedCmd(command.id, static_cast<uint16_t>(command.value)));
    break;
  case UserCmd::Type::Direction:
    push(DirectionCmd(command.id, command.flag));
    break;
  case UserCmd::Type::Switch:
    push(SwitchCmd(static_cast<uint16_t>(command.id), command.flag));
    break;
  case UserCmd::Type::Reverse: {
    auto loco = state.get_loco(command.id);
    push(SpeedCmd(command.id, 0));
    push(DirectionCmd(command.id, !loco.backward));
    push(SpeedCmd(command.id, loco.requested_speed));
    break;
  }
  case UserCmd::Type::Stop:
    push(ControlCmd(ControlCmd::CMD_STOP));
    break;
  case UserCmd::Type::Go:
    push(ControlCmd(ControlCmd::CMD_GO));
    break;
  case UserCmd::Type::Reset: {
    State default_state{};

    push(ControlCmd(ControlCmd::CMD_HALT));
    for (const Train &train : default_state.trains) {
      push(LightCmd(train.loco_id, train.light_on));
      push(SpeedCmd(train.loco_id, train.requested_speed));
      push(DirectionCmd(train.loco_id, train.backward));
    }

    for (int sw_id = 0; sw_id < 22; ++sw_id) {
      push(SwitchCmd(State::switch_id(sw_id), default_state.is_switch_straight(
                                                  State::switch_id(sw_id))));
    }

    push(ControlCmd(default_state.stopped ? ControlCmd::CMD_STOP
                                          : ControlCmd::CMD_GO));
    break;
  }
  case UserCmd::Type::Invalid:
  case UserCmd::Type::Quit:
    break;
  }

  return command_count;
}

template <> void TrainControlServer<>::rx_can_worker() {
  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  CANFRAME frame{};
  TC::RX msg{};
  while (true) {
    await_event(Event::CAN_RX_IRQ);
    rx_can(frame);
    msg.mrk         = decode_frame(frame);
    auto cans_reply = send<TC::Ack>(tcs_tid, msg);
    if (!cans_reply.has_value()) {
      break;
    }
  }
}
