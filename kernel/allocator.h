#pragma once

#include <optional>

#include "buffer.h"

template <size_t SIZE> class Allocator {
  Buffer<int, SIZE> free_items;

public:
  // constexpr so we don't have to initalize at runtime
  constexpr Allocator() {
    for (size_t i = 0; i < SIZE; i++) {
      free_items.push(i);
    }
  }

  constexpr std::optional<int> allocate() {
    auto item = free_items.peek();
    if (!item.has_value()) {
      return std::nullopt;
    }
    free_items.pop();
    return item.value();
  }

  constexpr bool free(int item) {
    if (item < 0 || static_cast<size_t>(item) >= SIZE) {
      return false; // invalid item
    }
    return free_items.push(item);
  }

  constexpr size_t allocated_count() const { return SIZE - free_items.size(); }
};
