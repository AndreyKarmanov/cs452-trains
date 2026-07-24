#include "heap.h"
#include "debug.h"

void test_heap() {
  {
    Heap<int, 8> heap;
    _assert(heap.empty(), "Expected new heap to be empty");
    _assert(!heap.peek().has_value(), "Expected empty heap peek to be nullopt");

    _assert(heap.push(4), "Expected push to succeed");
    _assert(heap.push(1), "Expected push to succeed");
    _assert(heap.push(7), "Expected push to succeed");
    _assert(heap.push(3), "Expected push to succeed");
    _assert(heap.push(2), "Expected push to succeed");
    _assert(heap.push(6), "Expected push to succeed");
    _assert(heap.push(5), "Expected push to succeed");

    _assert(heap.size() == 7, "Expected heap size to be 7");
    _assert(heap.peek().has_value() && heap.peek().value() == 1,
            "Expected min-heap peek to be 1");

    const int expected[] = {1, 2, 3, 4, 5, 6, 7};
    for (size_t i = 0; i < 7; ++i) {
      auto value = heap.pop();
      _assert(value.has_value(), "Expected pop to return a value");
      _assert(value.value() == expected[i], "Expected sorted pop order");
    }

    _assert(heap.empty(), "Expected heap to be empty after pops");
    _assert(!heap.pop().has_value(), "Expected pop on empty heap to fail");
  }

  {
    Heap<int, 8, std::greater<int>> heap;
    heap.push(4);
    heap.push(1);
    heap.push(7);
    heap.push(3);

    auto first  = heap.pop();
    auto second = heap.pop();
    _assert(first.has_value() && first.value() == 7,
            "Expected max-heap first pop to be 7");
    _assert(second.has_value() && second.value() == 4,
            "Expected max-heap second pop to be 4");
    _assert(heap.peek().has_value() && heap.peek().value() == 3,
            "Expected max-heap peek to be 3 after two pops");
  }
}
