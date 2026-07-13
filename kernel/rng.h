#pragma once

#include "io_helpers.h"
#include "uart_tx_server.h"
#include <array>
#include <cstdint>

class RNG {
  // https://www.ams.org/journals/mcom/1999-68-225/S0025-5718-99-00996-5/S0025-5718-99-00996-5.pdf
  uint32_t state = 42;
  uint32_t a     = 438293613;
  uint32_t c     = 1;       // must be odd for full period with def params
  uint32_t m     = 1 << 30; // 2^30, assume power of 2

public:
  RNG() = default;
  RNG(uint32_t seed) : state(seed) {};
  RNG(uint32_t seed, uint32_t a, uint32_t c, uint32_t m_pow)
      : state(seed), a(a), c(c), m(1 << m_pow) {
          // could check that a,c,m guarantee best period but need primes and
          // stuff and easier to do offline.
        };
  uint32_t nextNum() {
    state = (a * state + c) & (m - 1);
    return state;
  };
};

class Unif : public RNG {
  uint32_t min;
  uint32_t max;

public:
  Unif(uint32_t seed, uint32_t min, uint32_t max)
      : RNG(seed), min(min), max(max) {};
  uint32_t nextNum() { return min + (RNG::nextNum() % (max - min + 1)); };
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

  for (int i = MIN_VAL; i < MAX_VAL; i++) {
    Debug_Puts(txs_tid, "Num ", i + 1, " count: ", counts[i]);
  }

  return true;
}