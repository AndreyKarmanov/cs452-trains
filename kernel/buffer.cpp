#include "buffer.h"

void test_buffer() {
  Buffer<int, 5> buf;

  assert(buf.is_empty(), "Non-empty on start");

  for (size_t i = 0; i < 5; ++i) {
    assert(buf.push(i), "Push Failed");
  }

  assert(buf.size() == 5, "size not consistent");

  for (int i = 0; i < 5; ++i) {
    assert(buf.peek() == i, "elem not correct");
    assert(buf.pop(), "elem failed to pop");
  }

  assert(buf.is_empty(), "buf not emptied");
}
