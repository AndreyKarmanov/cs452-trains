#pragma once

#include "uart_tx_server.h"
#include <array>
#include <cstdint>

class RNG {
  // https://www.ams.org/journals/mcom/1999-68-225/S0025-5718-99-00996-5/S0025-5718-99-00996-5.pdf
  uint64_t state = 42;
  uint64_t a     = 438293613;
  uint64_t c     = 1;       // must be odd for full period with def params
  uint64_t m     = 1 << 30; // 2^30, assume power of 2

public:
  RNG() = default;
  RNG(uint64_t seed) : state(seed) {};
  RNG(uint64_t seed, uint64_t a, uint64_t c, uint64_t m_pow)
      : state(seed), a(a), c(c), m(1 << m_pow) {
          // could check that a,c,m guarantee best period but need primes and
          // stuff and easier to do offline.
        };
  uint64_t nextNum() {
    state = (a * state + c) & (m - 1);
    return state;
  };
};

class Unif : public RNG {
  uint64_t min;
  uint64_t max;

public:
  Unif(uint64_t seed, uint64_t min, uint64_t max)
      : RNG(seed), min(min), max(max) {};
  uint64_t nextNum() { return min + (RNG::nextNum() % (max - min + 1)); };
};

inline bool test_rng() {
  constexpr static int MAX_VAL     = 10;
  constexpr static int NUM_SAMPLES = 10000;
  constexpr static int MIN_VAL     = 1;

  Unif u(42, MIN_VAL, MAX_VAL);
  std::array<int, MAX_VAL> counts{};
  auto txs_tid = WhoIs(UART_TX_Server::NAME);

  for (int i = 0; i < NUM_SAMPLES; i++) {
    auto n = u.nextNum();
    if (n < MIN_VAL || n > MAX_VAL) {
      Debug_Puts(txs_tid,
                 "RNG test failed: generated number out of range: ", n);
      return false;
    }
    counts[n - MIN_VAL]++;
  }

  for (int i = MIN_VAL - 1; i < MAX_VAL; i++) {
    Debug_Puts(txs_tid, "Num ", i + 1, " count: ", counts[i]);
  }

  return true;
}