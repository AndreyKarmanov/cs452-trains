#include "io_helpers.h"
#include "message.h"
#include "syscall.h"

// tid should be the RX server tid
int Getc(int tid) {
  auto rcv_msg = send<RX::GetcReplyMsg>(tid, RX::GetcMsg{});
  if (!rcv_msg.has_value()) {
    return -1;
  }
  return static_cast<int>(rcv_msg->c);
}
