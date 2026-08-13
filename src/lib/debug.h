#pragma once

#include <cstddef>
#include <source_location>

bool _assert(bool condition, const char *msg,
             const std::source_location loc = std::source_location::current());

void panic_if(bool condition, const char *msg,
              const std::source_location loc = std::source_location::current());

void dump_memory_region(size_t address, size_t count);
void write_memory_word(size_t address, size_t value);
