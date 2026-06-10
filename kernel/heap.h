#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <utility>

template <typename T, size_t MAX_SIZE, typename Compare = std::less<T>>
class Heap {
  std::array<std::optional<T>, MAX_SIZE> buf;
  Compare compare{};
  size_t count = 0;

public:
  std::optional<T> peek() const {
    if (is_empty())
      return std::nullopt;
    return buf[0];
  }

  std::optional<T> pop() {
    if (is_empty())
      return std::nullopt;
    if (count == 1) {
      --count;
      return buf[0];
    }

    auto ret   = buf[0];
    buf[0]     = buf[--count];
    size_t idx = 0;

    // bubble down
    while (true) {
      size_t best = idx * 2 + 1;
      if (best >= count)
        break;

      size_t right = best + 1;
      if (right < count && compare(*buf[right], *buf[best]))
        best = right;

      if (!compare(*buf[best], *buf[idx]))
        break;

      std::swap(buf[best], buf[idx]);
      idx = best;
    }

    return ret;
  }

  bool push(const T &v) {
    if (count == MAX_SIZE)
      return false;

    buf[count] = v;
    size_t idx = count++;

    // bubble up
    while (idx != 0) {
      size_t p_idx = (idx - 1) / 2;
      if (compare(*buf[idx], *buf[p_idx])) {
        std::swap(buf[idx], buf[p_idx]);
        idx = p_idx;
      } else {
        break;
      }
    }

    return true;
  }

  size_t size() const { return count; }
  bool is_empty() const { return count == 0; }
};