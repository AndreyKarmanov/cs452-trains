#include "rps_server.h"

void RPSServer::run() {}

void rps_server_task() {
  RPSServer server{};
  server.run();
}
