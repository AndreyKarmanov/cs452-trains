#pragma once

#include <cstdint>
#include <stdint.h>

#define TIME_1S_US 1'000'000
#define TICK_TIME_US 1'000
#define TICKS_PER_S (TIME_1S_US / TICK_TIME_US)
#define TICKS_PER_MS (uint32_t)(TICK_TIME_US / 1000)

void set_timer_interrupt(uint32_t timer, uint32_t delay_us);
void update_timer_interrupt(uint32_t timer, uint32_t delta_us);
void clear_timer_interrupt(uint32_t timer);

uint32_t time_get();
const char *format_time(uint32_t time_us);