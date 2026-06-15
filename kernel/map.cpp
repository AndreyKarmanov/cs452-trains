#include "map.h"
#include "debug.h"
#include "static_string.h"

struct ZeroHasher {
  constexpr size_t operator()(const StaticString<16> &) const { return 0; }
};

void test_map() {
  StaticString<16> key1("hello");
  StaticString<16> key2("world");
  StaticString<16> key3("there");

  Map<StaticString<16>, int, 16> map;

  map.set(key1, 42);
  map.set(key2, 84);

  auto val1 = map.get(key1);
  auto val2 = map.get(key2);

  _assert(val1.has_value(), "Expected key 'hello' to be present in the map");
  _assert(val1.has_value() && val1.value() == 42,
          "Expected value 42 for key 'hello'");
  _assert(val2.has_value() && val2.value() == 84,
          "Expected value 84 for key 'world'");

  _assert(map.size() == 2, "Expected map size to be 2");
  map.remove(key1);

  _assert(!map.get(key1).has_value(), "Expected key 'hello' to be removed");
  _assert(map.get(key2).has_value() && map.get(key2).value() == 84,
          "Expected key 'world' to still be present with value 84");

  _assert(map.size() == 1, "Expected map size to be 1 after removal");

  Map<StaticString<16>, int, 16, ZeroHasher> colliding_map;
  colliding_map.set(key1, 1);
  colliding_map.set(key2, 2);
  colliding_map.set(key3, 3);

  colliding_map.remove(key2);

  _assert(colliding_map.get(key1).has_value() &&
              colliding_map.get(key1).value() == 1,
          "Expected first colliding key to remain reachable after deletion");
  _assert(colliding_map.get(key3).has_value() &&
              colliding_map.get(key3).value() == 3,
          "Expected later colliding key to remain reachable after deletion");
  _assert(!colliding_map.get(key2).has_value(),
          "Expected removed colliding key to be absent");

  // Test iterator with structured bindings
  Map<StaticString<16>, int, 16> iter_map;
  iter_map.set(key1, 10);
  iter_map.set(key2, 20);
  iter_map.set(key3, 30);

  int count = 0;
  for (const auto &[k, v] : iter_map) {
    count++;
    if (k == key1) {
      _assert(v == 10, "Expected value 10 for key1");
    } else if (k == key2) {
      _assert(v == 20, "Expected value 20 for key2");
    } else if (k == key3) {
      _assert(v == 30, "Expected value 30 for key3");
    }
  }
  _assert(count == 3, "Expected iterator to visit all 3 entries");
}