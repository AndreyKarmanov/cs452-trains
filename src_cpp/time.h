#ifndef _time_h_
#define _time_h_ 1

#include <stdint.h>

uint32_t time_get();
const char* format_time(uint32_t time_us);
void print_time(uint32_t time_us);

#endif /* time.h */