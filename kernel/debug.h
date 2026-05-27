#ifndef _debug_h_
#define _debug_h_ 1

#include <stddef.h>
#include <stdint.h>

// #include "can.h"

// void debug_put_bin8(size_t line, uint8_t value);
// void debug_put_bin32(size_t line, uint32_t value);

// void debug_print_memory_dump(const void* start, uint32_t nbytes);
// void debug_print_memory_bits(const void* start, uint32_t nbytes);
// void debug_print_can_frame(const CANFRAME* frame);
// void debug_print_mrk(const MRK_CMD& cmd);

bool assert(bool value, const char* msg);

#endif /* _debug_h_ */
