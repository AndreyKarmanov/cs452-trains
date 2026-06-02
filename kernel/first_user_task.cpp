#include "first_user_task.h"
#include "name_server.h"
#include "rps_client.h"
#include "rps_server.h"
#include "syscall.h"

void first_user_task() {
  create(2, name_server_task);
  create(2, rps_server_task);
  for (size_t i = 0; i < 5; ++i) {
    create(3, rps_client_task);
  }
}