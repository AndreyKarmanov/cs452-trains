#pragma once

#include <cstdint>
#include <stdint.h>

#define TIME_1S_US 1'000'000
#define TICK_TIME_US 1'000
#define TICKS_IN_1S (TIME_1S_US / TICK_TIME_US)

void set_timer_interrupt(uint32_t timer, uint32_t delay_us);
void update_timer_interrupt(uint32_t timer, uint32_t delta_us);
void clear_timer_interrupt(uint32_t timer);

uint32_t time_get();
const char *format_time(uint32_t time_us);