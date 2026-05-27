#ifndef _buffer_h_
#define _buffer_h_ 1

#include <stddef.h>

#include "debug.h"

template <typename T, size_t SIZE>
class Buffer {
  T arr[SIZE];
  size_t head = 0;
  size_t _size = 0;

 public:
  bool push(T elem) {
    if (_size == SIZE) return false;

    arr[(head + _size) % SIZE] = elem;
    ++_size;

    return true;
  }

  T peek_last() const {
    if (_size == 0) return T();
    return arr[(head + _size - 1) % SIZE];
  }

  bool pop() {
    if (is_empty()) return false;

    head = (head + 1) % SIZE;
    --_size;

    return true;
  }

  T peek() const { return arr[head]; }

  inline bool is_empty() const { return _size == 0; }

  inline size_t size() const { return _size; }

  T operator[](size_t i) const { return arr[(head + i) % SIZE]; }

  class Iterator {
    const Buffer* buf;
    size_t i;  // offset from head
   public:
    Iterator(const Buffer* b, size_t i) : buf(b), i(i) {}
    bool operator!=(const Iterator& other) const { return i != other.i; }
    Iterator& operator++() {
      ++i;
      return *this;
    }
    T operator*() const { return buf->arr[(buf->head + i) % SIZE]; }
  };

  Iterator begin() const { return Iterator(this, 0); }
  Iterator end() const { return Iterator(this, _size); }
};

void test_buffer();
#endif  // buffer.h
