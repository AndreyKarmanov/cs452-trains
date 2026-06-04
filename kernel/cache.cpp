#include "cache.h"
#include <cstdint>

void flush_cache() {}

void data_caching_set(bool enabled) {
  // set bit 2 of the SCTLR_EL1 register
  // need to clean out the cache to make sure there' no lost data
  uint64_t SCTLR_EL1;
  asm volatile("mrs %0, SCTLR_EL1" : "=r"(SCTLR_EL1));
  bool prev_state = SCTLR_EL1 & (1 << 2);

  if (enabled == prev_state) {
    return;
  }

  if (enabled) {
    SCTLR_EL1 |= (1 << 2);
  } else {
    SCTLR_EL1 &= ~(1 << 2);
  }

  // need to clean out the cache here?
  // if enabled -> disabled, need to write back
  // if disabled -> enabled, need to reset to avoid stale data
  // updating to the same state don't do anything?
  asm volatile("msr SCTLR_EL1, %0" : : "r"(SCTLR_EL1));
}

void instruction_cache_set(bool enabled) {
  // set bit 12 of the SCTLR_EL1 register
  // don't need to worry about data, as this is instructions
  uint64_t SCTLR_EL1;
  asm volatile("mrs %0, SCTLR_EL1" : "=r"(SCTLR_EL1));
  bool prev_state = SCTLR_EL1 & (1 << 12);

  if (enabled == prev_state) {
    return;
  }
  if (enabled) {
    SCTLR_EL1 |= (1 << 12);
  } else {
    SCTLR_EL1 &= ~(1 << 12);
  }
  asm volatile("msr SCTLR_EL1, %0" : : "r"(SCTLR_EL1));
}