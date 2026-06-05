#pragma once

#include <cstddef>
#include <functional>

template <size_t SIZE> struct StaticString {
  char data[SIZE]{};
  size_t len{0};

  StaticString() = default;
  StaticString(const char *str) {
    while (len < SIZE - 1 && str[len] != '\0') {
      data[len] = str[len];
      len++;
    }
    data[len] = '\0';
  }

  constexpr bool operator==(const StaticString &other) const {
    if (len != other.len)
      return false;
    for (size_t i = 0; i < len; ++i) {
      if (data[i] != other.data[i])
        return false;
    }
    return true;
  }
};

namespace std {
  template <size_t SIZE> struct hash<StaticString<SIZE>> {
  public:
    constexpr size_t operator()(const StaticString<SIZE> &name_key) const {

      // https://cppreference.com/cpp/utility/hash/operator%28%29
      // Computes the hash of an employee using Fowler-Noll-Vo-1a hash function.
      constexpr std::size_t prime{sizeof(size_t) < 8 ? 0x01000193
                                                     : 0x100000001b3};
      std::size_t result{sizeof(size_t) < 8 ? 0x811c9dc5 : 0xcbf29ce484222325};

      for (size_t i = 0; i < name_key.len; ++i) {
        result =
            (result ^ static_cast<unsigned char>(name_key.data[i])) * prime;
      }

      return result;
    }
  };
} // namespace std
