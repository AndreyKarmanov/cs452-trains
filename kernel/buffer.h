#pragma once

#include <array>
#include <cstddef>
#include <optional>
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
  constexpr bool push(T elem) {
    if (_size == SIZE)
      return false;

    arr[(head + _size) % SIZE] = elem;
    ++_size;

    return true;
  }

  constexpr std::optional<T> peek_last() const {
    if (_size == 0)
      return std::nullopt;
    return arr[(head + _size - 1) % SIZE];
  }

  constexpr std::optional<T> pop() {
    if (is_empty())
      return std::nullopt;
    auto elem = arr[head];

    head = (head + 1) % SIZE;
    --_size;

    return elem;
  }

  constexpr std::optional<T> peek() const {
    if (is_empty())
      return std::nullopt;
    return arr[head];
  }

  constexpr inline bool is_empty() const { return _size == 0; }

  constexpr inline size_t size() const { return _size; }

  constexpr std::optional<T> operator[](size_t i) const {
    if (i >= _size)
      return std::nullopt;
    return arr[(head + i) % SIZE];
  }

  class Iterator {
    const Buffer *buf;
    size_t i; // offset from head
  public:
    Iterator(const Buffer *b, size_t i) : buf(b), i(i) {}
    bool operator!=(const Iterator &other) const { return i != other.i; }
    Iterator &operator++() {
      ++i;
      return *this;
    }
    T operator*() const { return buf->arr[(buf->head + i) % SIZE]; }
  };

  Iterator begin() const { return Iterator(this, 0); }
  Iterator end() const { return Iterator(this, _size); }
};
