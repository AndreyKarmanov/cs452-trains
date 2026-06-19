#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

template <size_t SIZE> struct StaticString {
  char data[SIZE]{};
  size_t len{0};

private:
  bool append_unsigned(uint64_t value) {
    char buffer[32];
    size_t index = 0;

    do {
      buffer[index++]  = static_cast<char>('0' + (value % 10));
      value           /= 10;
    } while (value != 0 && index < sizeof(buffer));

    if (value != 0)
      return false;

    while (index > 0) {
      if (!append(buffer[--index]))
        return false;
    }
    return true;
  }

  bool append_signed(int64_t value) {
    if (value < 0) {
      if (!append('-'))
        return false;
      return append_unsigned(static_cast<uint64_t>(-(value + 1)) + 1);
    }
    return append_unsigned(static_cast<uint64_t>(value));
  }

public:
  StaticString() = default;
  StaticString(const char *str) {
    while (len < SIZE - 1 && str[len] != '\0') {
      data[len] = str[len];
      len++;
    }
    data[len] = '\0';
  }

  StaticString(const char *str, size_t length) {
    len = length < SIZE - 1 ? length : SIZE - 1;
    for (size_t i = 0; i < len; ++i) {
      data[i] = str[i];
    }
    data[len] = '\0';
  }

  void clear() {
    len     = 0;
    data[0] = '\0';
  }

  bool empty() const { return len == 0; }

  bool pop_back() {
    if (len == 0)
      return false;
    --len;
    data[len] = '\0';
    return true;
  }

  template <typename... Args> void set(const Args &...args) {
    clear();
    (append(args), ...);
  }

  bool append(char ch) {
    if (len >= SIZE - 1)
      return false;
    data[len++] = ch;
    data[len]   = '\0';
    return true;
  }

  bool append(const char *str) {
    while (*str != '\0') {
      if (!append(*str++))
        return false;
    }
    return true;
  }

  template <size_t OTHER_SIZE>
  bool append(const StaticString<OTHER_SIZE> &other) {
    return append(other.data);
  }

  bool append(unsigned int value) { return append_unsigned(value); }

  bool append(unsigned short value) { return append_unsigned(value); }

  bool append(unsigned char value) { return append_unsigned(value); }

  bool append(unsigned long value) { return append_unsigned(value); }

  bool append(unsigned long long value) { return append_unsigned(value); }

  bool append(int value) { return append_signed(value); }

  bool append(short value) { return append_signed(value); }

  bool append(signed char value) { return append_signed(value); }

  bool append(long value) { return append_signed(value); }

  bool append(long long value) { return append_signed(value); }

  bool append() { return true; }

  template <typename First, typename Second, typename... Rest>
  bool append(const First &first, const Second &second, const Rest &...rest) {
    if (!append(first))
      return false;
    if constexpr (sizeof...(Rest) == 0) {
      return append(second);
    } else {
      return append(second, rest...);
    }
  }

  const char *c_str() const { return data; }

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
