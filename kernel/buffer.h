#pragma once

#include "debug.h"
#include "ranges"
#include "uart.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <optional>
#include <type_traits>

// TODO: make a linked list version to enable removing arbitrary
template <typename T, size_t SIZE> class alignas(16) Buffer {
  std::array<T, SIZE> arr;
  size_t head  = 0;
  size_t _size = 0;

public:
  // constexpr allows us to create default buffers at compile time
  // e.g. we can do
  // `constexpr make_buffer() { for (...) buf.push(T{}); return buf; }`
  // and it will have no runtime cost
  constexpr bool push(const T &elem) {
    if (_size == SIZE)
      return false;

    arr[(head + _size) % SIZE] = elem;
    ++_size;

    return true;
  }

  constexpr bool push_front(const T &elem) {
    if (_size == SIZE)
      return false;

    head      = (head + SIZE - 1) % SIZE;
    arr[head] = elem;
    ++_size;

    return true;
  }

  constexpr std::optional<T> peek_last() const {
    if (_size == 0)
      return std::nullopt;
    return arr[(head + _size - 1) % SIZE];
  }

  constexpr void pop(size_t n) {
    n      = std::min(n, _size);
    head   = (head + n) % SIZE;
    _size -= n;
  }

  constexpr std::optional<T> pop() {
    if (empty())
      return std::nullopt;
    auto elem = arr[head];

    head = (head + 1) % SIZE;
    --_size;

    return elem;
  }

  constexpr std::optional<T> peek() const {
    if (empty())
      return std::nullopt;
    return arr[head];
  }

  constexpr inline bool empty() const { return _size == 0; }

  constexpr inline size_t size() const { return _size; }

  constexpr inline size_t capacity() const { return SIZE; }

  constexpr std::optional<T> operator[](size_t i) const {
    if (i >= _size)
      return std::nullopt;
    return arr[(head + i) % SIZE];
  }

  template <bool IsConst> class IteratorBase {
    using BufferType = std::conditional_t<IsConst, const Buffer, Buffer>;

    BufferType *buf = nullptr;
    size_t i        = 0; // offset from head

  public:
    using element_type    = T;
    using difference_type = std::ptrdiff_t;
    using reference       = std::conditional_t<IsConst, const T &, T &>;

    constexpr IteratorBase() = default;
    constexpr IteratorBase(BufferType *b, size_t index) : buf(b), i(index) {}

    constexpr bool operator==(const IteratorBase &) const = default;

    constexpr reference operator*() const {
      return buf->arr[(buf->head + i) % SIZE];
    }

    constexpr IteratorBase &operator++() {
      ++i;
      return *this;
    }

    constexpr IteratorBase operator++(int) {
      auto copy = *this;
      ++(*this);
      return copy;
    }

    constexpr IteratorBase &operator--() {
      --i;
      return *this;
    }

    constexpr IteratorBase operator--(int) {
      auto copy = *this;
      --(*this);
      return copy;
    }

    constexpr IteratorBase &operator+=(difference_type n) {
      if (n >= 0)
        i += static_cast<size_t>(n);
      else
        i -= static_cast<size_t>(-n);
      return *this;
    }

    constexpr IteratorBase operator+(difference_type n) const {
      auto copy  = *this;
      copy      += n;
      return copy;
    }

    friend constexpr IteratorBase operator+(difference_type n,
                                            const IteratorBase &it) {
      return it + n;
    }

    constexpr IteratorBase &operator-=(difference_type n) {
      return *this += -n;
    }

    constexpr IteratorBase operator-(difference_type n) const {
      auto copy  = *this;
      copy      -= n;
      return copy;
    }

    constexpr difference_type operator-(const IteratorBase &other) const {
      return static_cast<difference_type>(static_cast<std::ptrdiff_t>(i) -
                                          static_cast<std::ptrdiff_t>(other.i));
    }

    constexpr reference operator[](difference_type n) const {
      return *(*this + n);
    }

    constexpr bool operator<(const IteratorBase &other) const {
      return i < other.i;
    }
    constexpr bool operator>(const IteratorBase &other) const {
      return i > other.i;
    }
    constexpr bool operator<=(const IteratorBase &other) const {
      return i <= other.i;
    }
    constexpr bool operator>=(const IteratorBase &other) const {
      return i >= other.i;
    }
  };

  using Iterator             = IteratorBase<false>;
  using ConstIterator        = IteratorBase<true>;
  using ReverseIterator      = std::reverse_iterator<Iterator>;
  using ConstReverseIterator = std::reverse_iterator<ConstIterator>;

  static_assert(std::forward_iterator<Iterator>);
  static_assert(std::bidirectional_iterator<Iterator>);
  static_assert(std::random_access_iterator<Iterator>);

  constexpr Iterator begin() { return Iterator(this, 0); }
  constexpr Iterator end() { return Iterator(this, _size); }
  constexpr ConstIterator begin() const { return ConstIterator(this, 0); }
  constexpr ConstIterator end() const { return ConstIterator(this, _size); }
  constexpr ConstIterator cbegin() const { return begin(); }
  constexpr ConstIterator cend() const { return end(); }

  constexpr ReverseIterator rbegin() { return ReverseIterator(end()); }
  constexpr ReverseIterator rend() { return ReverseIterator(begin()); }
  constexpr ConstReverseIterator rbegin() const {
    return ConstReverseIterator(end());
  }
  constexpr ConstReverseIterator rend() const {
    return ConstReverseIterator(begin());
  }
  constexpr ConstReverseIterator crbegin() const { return rbegin(); }
  constexpr ConstReverseIterator crend() const { return rend(); }
};

inline void test_buffer() {
  Buffer<int, 4> buf{};
  _assert(buf.push(1), "buffer push 1 failed");
  _assert(buf.push(2), "buffer push 2 failed");
  _assert(buf.push(3), "buffer push 3 failed");

  int expected_forward[] = {1, 2, 3};
  int index              = 0;
  for (auto it = buf.begin(); it != buf.end(); ++it) {
    _assert(*it == expected_forward[index], "buffer forward iteration failed");
    ++index;
  }
  _assert(index == 3, "buffer forward iteration count failed");

  const auto &const_buf = buf;
  index                 = 0;
  for (auto it = const_buf.cbegin(); it != const_buf.cend(); ++it) {
    _assert(*it == expected_forward[index], "buffer const iteration failed");
    ++index;
  }
  _assert(index == 3, "buffer const iteration count failed");

  int expected_reverse[] = {3, 2, 1};
  index                  = 0;
  for (auto it = buf.rbegin(); it != buf.rend(); ++it) {
    _assert(*it == expected_reverse[index], "buffer reverse iteration failed");
    ++index;
  }
  _assert(index == 3, "buffer reverse iteration count failed");

  index = 0;
  for (auto it = const_buf.crbegin(); it != const_buf.crend(); ++it) {
    _assert(*it == expected_reverse[index],
            "buffer const reverse iteration failed");
    ++index;
  }
  _assert(index == 3, "buffer const reverse iteration count failed");

  for (auto ele : std::views::reverse(const_buf)) {
    debug_printf(CONSOLE, "%d ", ele);
  }

  debug_puts(CONSOLE, "\n\rBuffer iteration test passed\n\r");
}