#pragma once

#include <cstdint>
#include <stdint.h>

#define TIME_1S_US 1000000
#define TIME_10MS_US 10000

void set_timer_interrupt(uint32_t timer, uint32_t delay_us);
void update_timer_interrupt(uint32_t timer, uint32_t delta_us);
void clear_timer_interrupt(uint32_t timer);

uint32_t time_get();
const char *format_time(uint32_t time_us);
// void print_time(uint32_t time_us);
