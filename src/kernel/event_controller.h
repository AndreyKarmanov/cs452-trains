#pragma once

#include "buffer.h"
#include "kernel/constants.h"
#include "map.h"
#include "syscall.h"
#include <cstdint>

struct EventController {
  uint64_t initialized_events{0};
  Map<Event, Buffer<int, MAX_TASKS>, TOTAL_EVENT_TYPES> event_buffers;

  void initialize_event(Event event);
  void uninitialize_event(Event event);
  void await_event(Event event, TaskId tid);
  void handle_event(Event event, int arg0 = 0);
};
