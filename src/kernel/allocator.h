#pragma once

#include "buffer.h"
#include <optional>

template <size_t SIZE> class Allocator : protected Buffer<size_t, SIZE> {

public:
  // constexpr so we don't have to initalize at runtime
  constexpr Allocator() {
    for (size_t i = 0; i < SIZE; i++) {
      Buffer<size_t, SIZE>::push(i);
    }
  }

  constexpr std::optional<int> allocate() {
    auto item = Buffer<size_t, SIZE>::pop();
    if (!item.has_value()) {
      return std::nullopt;
    }
    return item.value();
  }

  constexpr bool free(size_t item) {
    if (item >= SIZE) {
      return false; // invalid item
    }
    return Buffer<size_t, SIZE>::push(item);
  }

  constexpr size_t allocated_count() const {
    return SIZE - Buffer<size_t, SIZE>::size();
  }
};
