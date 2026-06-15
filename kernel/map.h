#pragma once

#include <array>
#include <functional>
#include <optional>

template <typename K, typename V, size_t SIZE, typename Hasher = std::hash<K>>
class Map {
public:
  struct Slot {
    K key;
    V value;
    enum class State { EMPTY, OCCUPIED, DELETED } state = State::EMPTY;
  };

  // Iterator for range-based for loops
  class iterator {
  private:
    Slot *slots;
    size_t idx;
    size_t max_size;

    constexpr void skip_empty() {
      while (idx < max_size && slots[idx].state != Slot::State::OCCUPIED) {
        ++idx;
      }
    }

  public:
    constexpr iterator(Slot *slots, size_t idx, size_t max_size)
        : slots(slots), idx(idx), max_size(max_size) {
      skip_empty();
    }

    constexpr std::pair<K &, V &> operator*() const {
      return {slots[idx].key, slots[idx].value};
    }

    constexpr iterator &operator++() {
      ++idx;
      skip_empty();
      return *this;
    }

    constexpr bool operator==(const iterator &other) const {
      return idx == other.idx;
    }

    constexpr bool operator!=(const iterator &other) const {
      return idx != other.idx;
    }
  };

private:
  std::array<Slot, SIZE> map;
  Hasher hasher;
  size_t count = 0;

  static constexpr bool is_occupied(const Slot &slot) {
    return slot.state == Slot::State::OCCUPIED;
  }

public:
  constexpr std::optional<V> get(const K &key) const {
    auto hash = hasher(key);
    auto idx  = hash % SIZE;
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (slot.state == Slot::State::EMPTY)
        return std::nullopt;
      if (is_occupied(slot) && slot.key == key)
        return slot.value;
      idx = (idx + 1) % SIZE;
    }
    return std::nullopt;
  }

  constexpr V *get_ref(const K &key) {
    auto hash = hasher(key);
    auto idx  = hash % SIZE;
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (slot.state == Slot::State::EMPTY)
        return nullptr;
      if (is_occupied(slot) && slot.key == key)
        return &slot.value;
      idx = (idx + 1) % SIZE;
    }
    return nullptr;
  }

  constexpr bool set(const K &key, const V &value) {
    auto hash                = hasher(key);
    auto idx                 = hash % SIZE;
    size_t first_deleted_idx = SIZE;
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (slot.state == Slot::State::OCCUPIED && slot.key == key) {
        slot.key   = key;
        slot.value = value;
        slot.state = Slot::State::OCCUPIED;
        return true;
      }

      // we can't put in the first deleted slot, until we confirm the key is not
      // already present later in the probe
      if (slot.state == Slot::State::DELETED && first_deleted_idx == SIZE) {
        first_deleted_idx = idx;
      } else if (slot.state == Slot::State::EMPTY) {
        // use first deleted slot if it exists, otherwise empty slot
        auto &target =
            first_deleted_idx == SIZE ? slot : map[first_deleted_idx];
        if (target.state != Slot::State::OCCUPIED)
          count++;
        target.key   = key;
        target.value = value;
        target.state = Slot::State::OCCUPIED;
        return true;
      }
      idx = (idx + 1) % SIZE;
    }

    if (first_deleted_idx != SIZE) {
      auto &target = map[first_deleted_idx];
      target.key   = key;
      target.value = value;
      target.state = Slot::State::OCCUPIED;
      count++;
      return true;
    }
    return false;
  }

  constexpr void remove(const K &key) {
    auto hash = hasher(key);
    auto idx  = hash % SIZE;
    for (size_t i = 0; i < SIZE; ++i) {
      auto &slot = map[idx];
      if (slot.state == Slot::State::EMPTY)
        return;
      if (is_occupied(slot) && slot.key == key) {
        count--;
        slot.state = Slot::State::DELETED;
        return;
      }
      idx = (idx + 1) % SIZE;
    }
  }

  constexpr size_t size() const { return count; }
  constexpr bool contains(const K &key) const { return get(key).has_value(); }

  constexpr iterator begin() { return iterator(map.data(), 0, SIZE); }

  constexpr iterator end() { return iterator(map.data(), SIZE, SIZE); }
};

void test_map();