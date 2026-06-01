#pragma once

#include <array>
#include <optional>

template <typename K, typename V, size_t SIZE> class BasicMap {
  std::array<std::pair<K, V>, SIZE> arr;
  size_t _size = 0;

public:
  constexpr bool set(const K &key, const V &value) {
    if (_size == SIZE)
      return false;
    arr[_size] = std::make_pair(key, value);
    ++_size;
    return true;
  }

  constexpr std::optional<V> get(const K &key) const {
    for (size_t i = 0; i < _size; ++i) {
      if (arr[i].first == key)
        return arr[i].second;
    }
    return std::nullopt;
  }
};
