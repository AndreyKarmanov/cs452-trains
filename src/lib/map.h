#pragma once

#include <array>
#include <functional>
#include <iterator>
#include <optional>
#include <type_traits>

template <typename K, typename V, size_t SIZE, typename Hasher = std::hash<K>>
class Map {
public:
  struct Slot {
    std::optional<K> key;
    std::optional<V> value;
    enum class State { EMPTY, OCCUPIED, DELETED } state = State::EMPTY;
  };

  template <bool IsConst> class IteratorBase {
  private:
    using MapType  = std::conditional_t<IsConst, const Map, Map>;
    using SlotType = std::conditional_t<IsConst, const Slot, Slot>;

    MapType *owner;
    SlotType *slots;
    size_t idx;
    size_t max_size;

    constexpr void skip_empty() {
      while (idx < max_size && slots[idx].state != Slot::State::OCCUPIED) {
        ++idx;
      }
    }

  public:
    using value_type      = std::pair<K, V>;
    using difference_type = std::ptrdiff_t;
    using reference = std::pair<std::conditional_t<IsConst, const K &, K &>,
                                std::conditional_t<IsConst, const V &, V &>>;

    constexpr IteratorBase() = default;

    constexpr IteratorBase(MapType *owner, SlotType *slots, size_t idx,
                           size_t max_size)
        : owner(owner), slots(slots), idx(idx), max_size(max_size) {
      skip_empty();
    }

    constexpr reference operator*() const {
      return {*slots[idx].key, *slots[idx].value};
    }

    constexpr IteratorBase &operator++() {
      ++idx;
      skip_empty();
      return *this;
    }

    constexpr IteratorBase operator++(int) {
      auto copy = *this;
      ++(*this);
      return copy;
    }

    constexpr IteratorBase &operator--() {
      do {
        --idx;
      } while (idx > 0 && slots[idx].state != Slot::State::OCCUPIED);
      return *this;
    }

    constexpr IteratorBase operator--(int) {
      auto copy = *this;
      --(*this);
      return copy;
    }

    constexpr bool operator==(const IteratorBase &other) const {
      return owner == other.owner && idx == other.idx;
    }

    constexpr bool operator!=(const IteratorBase &other) const {
      return !(*this == other);
    }
  };

  using iterator               = IteratorBase<false>;
  using const_iterator         = IteratorBase<true>;
  using reverse_iterator       = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

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
      if (is_occupied(slot) && slot.key.has_value() && *slot.key == key)
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
      if (is_occupied(slot) && slot.key.has_value() && *slot.key == key)
        return &(*slot.value);
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
      if (slot.state == Slot::State::OCCUPIED && slot.key.has_value() &&
          *slot.key == key) {
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
      if (is_occupied(slot) && slot.key.has_value() && *slot.key == key) {
        count--;
        slot.state = Slot::State::DELETED;
        return;
      }
      idx = (idx + 1) % SIZE;
    }
  }

  constexpr void clear() {
    for (auto &slot : map) {
      slot.state = Slot::State::EMPTY;
      slot.key.reset();
      slot.value.reset();
    }
    count = 0;
  }

  constexpr bool operator==(const Map &other) const {
    if (count != other.count)
      return false;
    for (const auto &[key, value] : *this) {
      auto other_value = other.get(key);
      if (!other_value.has_value() || *other_value != value)
        return false;
    }
    return true;
  }

  constexpr size_t size() const { return count; }
  constexpr bool contains(const K &key) const { return get(key).has_value(); }

  constexpr iterator begin() { return iterator(this, map.data(), 0, SIZE); }

  constexpr iterator end() { return iterator(this, map.data(), SIZE, SIZE); }

  constexpr const_iterator begin() const {
    return const_iterator(this, map.data(), 0, SIZE);
  }

  constexpr const_iterator end() const {
    return const_iterator(this, map.data(), SIZE, SIZE);
  }

  constexpr const_iterator cbegin() const { return begin(); }
  constexpr const_iterator cend() const { return end(); }

  constexpr reverse_iterator rbegin() { return reverse_iterator(end()); }
  constexpr reverse_iterator rend() { return reverse_iterator(begin()); }

  constexpr const_reverse_iterator rbegin() const {
    return const_reverse_iterator(end());
  }

  constexpr const_reverse_iterator rend() const {
    return const_reverse_iterator(begin());
  }

  constexpr const_reverse_iterator crbegin() const { return rbegin(); }
  constexpr const_reverse_iterator crend() const { return rend(); }
};

void test_map();