#pragma once

#include <optional>

#include "buffer.h"

template <typename T, size_t SIZE> class Allocator {
  Buffer<T, SIZE> free_items;

public:
  // constexpr so we don't have to initalize at runtime
  constexpr Allocator() {
    for (size_t i = 0; i < SIZE; i++) {
      free_items.push(i);
    }
  }

  constexpr std::optional<T> allocate() {
    auto item = free_items.peek();
    if (!item.has_value()) {
      return std::nullopt;
    }
    free_items.pop();
    return item.value();
  }

  constexpr bool free(T item) {
    if (item < 0 || item >= SIZE) {
      return false; // invalid item
    }
    return free_items.push(item);
  }
};
