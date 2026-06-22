#pragma once

#include "buffer.h"
#include "debug.h"
#include "message.h"
#include "mrk.h"
#include "name_server.h"
#include "syscall.h"

class CAN_RxServer {
public:
  static constexpr auto CAN_RX_SERVER_NAME = "CANRXSERVER";
  static constexpr size_t FORWARD_BUFFER_SIZE = 64;

  CAN_RxServer() {
    auto response = RegisterAs(CAN_RX_SERVER_NAME);
    _assert(response == 0, "CAN RX SERVER REGISTERAS FAILED");
  }

  void run();

private:
  CANFRAME pending_frame{};
  bool has_pending     = false;
  bool response_ready  = false;
  int waiting_pace_tid = -1;
  int waiting_forward_tid = -1;

  Buffer<TC::RX, FORWARD_BUFFER_SIZE> forward_buffer;

  void clear_pacing_state();
  void try_reply_pace();
  void try_reply_forward_courier();
  void handle_received_frame(const CANFRAME &frame);

  void handle(int tid, const CRX::PaceRegisterMsg &msg);
  void handle(int tid, const CRX::PaceAwaitMsg &);
  void handle(int tid, const CRX::ForwardReadyMsg &);
  void handle(int tid, const CRX::InterruptMsg &);
  template <class T> void handle(int tid, const T &) { reply_with_error(tid); }
};

void can_rx_server_task();
