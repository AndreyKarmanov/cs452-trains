#include "can_rx_server.h"
#include "mcp2515.h"
#include "train_control.h"

static void can_rx_notifier_task() {
  int can_rx_tid = WhoIs(CAN_RxServer::CAN_RX_SERVER_NAME);
  _assert(can_rx_tid >= 0, "CAN RX SERVER WHOIS FAILED");

  while (true) {
    await_event(Event::CAN_RX_IRQ);
    auto rcv_msg =
        send<CRX::InterruptReplyMsg>(can_rx_tid, CRX::InterruptMsg{});
    if (!rcv_msg.has_value()) {
      break;
    }
  }
}

static void tc_forward_courier_task() {
  int can_rx_tid = WhoIs(CAN_RxServer::CAN_RX_SERVER_NAME);
  _assert(can_rx_tid >= 0, "CAN RX SERVER WHOIS FAILED");

  int tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER NOT FOUND");

  while (true) {
    // get next frame to forward
    auto fwd = send<CRX::ForwardMsg>(can_rx_tid, CRX::ForwardReadyMsg{});
    if (!fwd.has_value()) {
      break;
    }

    // forward the frame to tc server
    auto tcs_reply = send<TC::Ack>(tcs_tid, TC::RX{.mrk = fwd->mrk});
    _assert(tcs_reply.has_value(), "CAN RX FORWARD COURIER FAILED");
  }
}

void can_rx_server_task() {
  CAN_RxServer can_rx_server;
  create(2, can_rx_notifier_task);
  create(2, tc_forward_courier_task);
  while (true) {
    can_rx_server.run();
    yield();
  }
}

void CAN_RxServer::clear_pacing_state() {
  has_pending    = false;
  response_ready = false;
}

void CAN_RxServer::try_reply_pace() {
  if (!response_ready || waiting_pace_tid < 0) {
    return;
  }

  reply(waiting_pace_tid, CRX::AckMsg{});
  waiting_pace_tid = -1;
  clear_pacing_state();
}

void CAN_RxServer::try_reply_forward_courier() {
  if (waiting_forward_tid < 0 || forward_buffer.is_empty()) {
    return;
  }

  auto rx = forward_buffer.pop();
  _assert(rx.has_value(), "CAN RX FORWARD POP FAILED");

  reply(waiting_forward_tid, CRX::ForwardMsg{.mrk = rx->mrk});
  waiting_forward_tid = -1;
}

void CAN_RxServer::handle_received_frame(const CANFRAME &frame) {
  auto pushed = forward_buffer.push(TC::RX{.mrk = decode_frame(frame)});
  _assert(pushed, "CAN RX FORWARD BUFFER FULL");

  if (has_pending && is_mrk_response_to(pending_frame, frame)) {
    response_ready = true;
    try_reply_pace();
  }

  try_reply_forward_courier();
}

void CAN_RxServer::handle(const int tid, const CRX::PaceRegisterMsg &msg) {
  pending_frame  = msg.frame;
  has_pending    = true;
  response_ready = false;
  reply(tid, CRX::AckMsg{});
}

void CAN_RxServer::handle(const int tid, const CRX::PaceAwaitMsg &) {
  if (response_ready) {
    reply(tid, CRX::AckMsg{});
    clear_pacing_state();
    return;
  }

  waiting_pace_tid = tid;
}

void CAN_RxServer::handle(const int tid, const CRX::ForwardReadyMsg &) {
  if (!forward_buffer.is_empty()) {
    auto rx = forward_buffer.pop();
    _assert(rx.has_value(), "CAN RX FORWARD POP FAILED");

    reply(tid, CRX::ForwardMsg{.mrk = rx->mrk});
    return;
  }

  waiting_forward_tid = tid;
}

void CAN_RxServer::handle(const int tid, const CRX::InterruptMsg &) {
  CANFRAME frame{};
  while (mcp2515_rx_pending()) {
    rx_can(frame);
    handle_received_frame(frame);
  }

  reply(tid, CRX::InterruptReplyMsg{});
}

void CAN_RxServer::run() {
  int tid;
  Message msg{};
  receive(&tid, msg);
  std::visit([&](auto &&arg) { handle(tid, arg); }, msg);
}
