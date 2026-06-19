#include "cache.h"
#include <cstdint>

// void flush_cache() {
//   // CLIDR_EL1:
//   //  ICB, bits [32:30] number of cache levels
//   // CSSELR_EL1, Cache Size Selection
//   //  Register Level, bits [3:1]
//   // CCSIDR_EL1, Current Cache Size ID Register,
//   //  NumSets, bits [55:32] number of sets - 1
//   //  Associativity, bits [23:3] number of ways - 1
//   //  LineSize, bits [2:0] Log2(Number of bytes in cache line)) - 4.

//   uint64_t CLIDR_EL1;

//   uint64_t ccsidr_el1;

//   asm volatile("mrs %0, CCSIDR_EL1" : "=r"(ccsidr_el1));

//   uint64_t num_sets  = ((ccsidr_el1 >> 13) & 0xFFFFFF) + 1;
//   uint64_t num_ways  = ((ccsidr_el1 >> 3) & 0x1FFFF) + 1;
//   uint64_t line_size = (1 << ((ccsidr_el1 & 0b111) + 4));
//     Printf(CONSOLE, "LEVEL: %u, SETS: %u WAYS %u LINE_SIZE: %u\n\r", 0,
//                 num_sets, num_ways, line_size);
// }

bool data_cache_set(bool enabled) {
  // set bit 2 of the SCTLR_EL1 register
  // need to clean out the cache to make sure there' no lost data
  uint64_t SCTLR_EL1;
  asm volatile("mrs %0, SCTLR_EL1" : "=r"(SCTLR_EL1));
  bool prev_state = SCTLR_EL1 & (1 << 2);

  if (enabled == prev_state) {
    // Printf(CONSOLE, "Data cache already in desired state: %s\n\r",
    //             enabled ? "enabled" : "disabled");
    return false;
  }
  // Printf(CONSOLE, "Setting Data cache state: %s\n\r",
  //             enabled ? "enabled" : "disabled");

  if (enabled) {
    SCTLR_EL1 |= (1 << 2);
  } else {
    SCTLR_EL1 &= ~(1 << 2);
  }

  // need to clean out the cache here?
  // if enabled -> disabled, need to write back
  // if disabled -> enabled, need to reset to avoid stale data
  // updating to the same state don't do anything?
  asm volatile("dsb sy"); // make sure all previous memory touches are done
  asm volatile("msr SCTLR_EL1, %0" : : "r"(SCTLR_EL1));
  asm volatile("isb");
  return true;
}

bool instruction_cache_set(bool enabled) {
  // set bit 12 of the SCTLR_EL1 register
  // don't need to worry about data, as this is instructions
  uint64_t SCTLR_EL1;
  asm volatile("mrs %0, SCTLR_EL1" : "=r"(SCTLR_EL1));
  bool prev_state = SCTLR_EL1 & (1 << 12);

  if (enabled == prev_state) {
    // Printf(CONSOLE, "Instruction cache already in desired state:
    // %s\n\r",
    //             enabled ? "enabled" : "disabled");
    return false;
  }
  // Printf(CONSOLE, "Setting Instruction cache state: %s\n\r",
  // enabled ? "enabled" : "disabled");

  if (enabled) {
    SCTLR_EL1 |= (1 << 12);
  } else {
    SCTLR_EL1 &= ~(1 << 12);
  }
  asm volatile("msr SCTLR_EL1, %0" : : "r"(SCTLR_EL1));
  asm volatile("dsb sy");
  asm volatile("isb");
  return true;
}