#pragma once

#include <array>
#include <functional>
#include <optional>

#include "uart.h"

template <typename K, typename V, size_t SIZE, typename Hasher = std::hash<K>>
class Map {
public:
  struct Slot {
    K key;
    V value;
    bool used = false;
  };

private:
  std::array<Slot, SIZE> map;
  Hasher hasher;
  size_t count = 0;

public:
  constexpr std::optional<V> get(const K &key) const {
    auto hash = hasher(key);
    auto idx  = hash % SIZE;
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (!slot.used)
        return std::nullopt;
      if (slot.key == key)
        return slot.value;
      idx = (idx + 1) % SIZE;
    }
    return std::nullopt;
  }

  constexpr void set(const K &key, const V &value) {
    auto hash = hasher(key);
    auto idx  = hash % SIZE;
    uart_printf(CONSOLE, "Setting key at index %u\n\r", idx);
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (!slot.used || slot.key == key) {
        if (!slot.used)
          count++;
        slot.key   = key;
        slot.value = value;
        slot.used  = true;
        return;
      }
      idx = (idx + 1) % SIZE;
    }
  }

  constexpr void remove(const K &key) {
    auto hash = hasher(key);
    auto idx  = hash % SIZE;
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (!slot.used)
        return;
      if (slot.key == key) {
        count--;
        slot.used = false;
        return;
      }
      idx = (idx + 1) % SIZE;
    }
  }

  constexpr size_t size() const { return count; }
};

void test_map();