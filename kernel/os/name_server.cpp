#include "name_server.h"
#include "debug.h"
#include "message.h"
#include "syscall.h"

void name_server_task() {
  static NameServer server;
  for (;;) {
    server.run();
    yield();
  }
}

int RegisterAs(const StaticString<NS::MAX_NAME_LEN> &name) {
  NS::RegisterMsg msg{.name = name};
  auto res = send<NS::RegisterReplyMsg>(NAMESERVER_TID, Message{msg});
  if (!res) {
    return res.error();
  }
  return static_cast<int>(res->status);
}

int WhoIs(const StaticString<NS::MAX_NAME_LEN> &name) {
  NS::WhoIsMsg msg{.name = name};
  auto res = send<NS::WhoIsReplyMsg>(NAMESERVER_TID, Message{msg});
  if (!res) {
    return res.error();
  }
  return res->tid;
}

void test_name_server() {
  int me = my_tid();
  int r  = RegisterAs("Clock");
  _assert(r == 0, "RegisterAs Clock: %d (expect 0)\n\r");
  int tid = WhoIs("Clock");
  _assert(tid == me, "WhoIs Clock: %d (expect %d)\n\r");
  int r2 = RegisterAs("Test2");
  _assert(r2 == 0, "RegisterAs Test2: %d (expect 0)\n\r");
  tid = WhoIs("Test2");
  _assert(tid == me, "WhoIs Test2: %d (expect %d)\n\r");
  tid = WhoIs("Missing");
  _assert(tid == -1, "WhoIs Missing: %d (expect -1)\n\r");
}
