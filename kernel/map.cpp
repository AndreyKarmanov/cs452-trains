#include "map.h"
#include "debug.h"
#include "static_string.h"

void test_map() {
  StaticString<16> key1("hello");
  StaticString<16> key2("world");

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
}