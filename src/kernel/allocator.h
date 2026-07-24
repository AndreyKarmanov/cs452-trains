#pragma once

#include "buffer.h"
#include <optional>

template <size_t SIZE> class Allocator : protected Buffer<int, SIZE> {

public:
  // constexpr so we don't have to initalize at runtime
  constexpr Allocator() {
    for (size_t i = 0; i < SIZE; i++) {
      Buffer<int, SIZE>::push(i);
    }
  }

  constexpr std::optional<int> allocate() {
    auto item = Buffer<int, SIZE>::pop();
    if (!item.has_value()) {
      return std::nullopt;
    }
    return item.value();
  }

  constexpr bool free(int item) {
    if (item < 0 || static_cast<size_t>(item) >= SIZE) {
      return false; // invalid item
    }
    return Buffer<int, SIZE>::push(item);
  }

  constexpr size_t allocated_count() const { return SIZE - Buffer<int, SIZE>::size(); }
};
