#pragma once

#include <stdint.h>
#define TIME_1S_US 1000000

uint32_t time_get();
const char *format_time(uint32_t time_us);
void print_time(uint32_t time_us);
