#ifndef _buffer_h_
#define _buffer_h_ 1

#include "debug.h"
#include <stddef.h>

template<typename T, size_t SIZE>
class Buffer
{
    T arr[SIZE];
    size_t head = 0;
    size_t _size = 0;
public:
    bool push(T elem) {
        if (_size == SIZE)
            return false;

        arr[(head + _size) % SIZE] = elem;
        ++_size;

        return true;
    }

    bool pop() {
        if (is_empty())
            return false;

        head = (head + 1) % SIZE;
        --_size;

        return true;
    }

    T peek() const {
        return arr[head];
    }

    inline bool is_empty() const {
        return _size == 0;
    }

    inline size_t size() const {
        return _size;
    }

    class Iterator {
        const Buffer* buf;
        size_t index;
        size_t count;
    public:
        Iterator(const Buffer* b, size_t i, size_t c) : buf(b), index(i), count(c) {}
        bool operator!=(const Iterator& other) const { return count != other.count; }
        void operator++() { index = (index + 1) % SIZE; ++count; }
        T operator*() const { return buf->arr[index]; }
    };

    Iterator begin() const { return Iterator(this, head, 0); }
    Iterator end() const { return Iterator(this, (head + _size) % SIZE, _size); }
};

void test_buffer();
#endif // buffer.h