#pragma once

#include <cstddef>

bool _assert(bool value, const char *msg);
void dump_memory_region(size_t address, size_t count);
void write_memory_word(size_t address, size_t value);
