#pragma once

#include <cstring>
#include <optional>

#include "basic_map.h"

constexpr size_t MAX_NAME_LENGTH = 32;

struct NameKey {
  char data[MAX_NAME_LENGTH];
  bool operator==(const NameKey &other) const {
    return strncmp(data, other.data, MAX_NAME_LENGTH) == 0;
  }
};

constexpr static int TID = 0;

class NameServer {
private:
  constexpr static size_t MAX_NAMES = 16;
  BasicMap<NameKey, int, MAX_NAMES> name_to_tid;

public:
  void run();

  // 0 for success, -1 for failure
  int RegisterAs(const char *name, int tid);

  std::optional<int> WhoIs(const char *name);
};

int RegisterAs(const char *name);
int WhoIs(const char *name);