#include "time.h"
#include "rpi.h"
#include "uart.h"
#include <cstdint>
#include <stdint.h>

#define TIME_COL "1"
#define TIME_ROW "2"

static char *const TIME_BASE = reinterpret_cast<char *>(MMIO_BASE + 0x3000);

// TIME register offsets
static const uint32_t TIME_CS  = 0x00;
static const uint32_t TIME_CLO = 0x04;
static const uint32_t TIME_CHI = 0x08;
static const uint32_t TIME_C1  = 0x10;
static const uint32_t TIME_C3  = 0x18;

#define SYSTIME_REG(reg)                                                       \
  *reinterpret_cast<volatile uint32_t *>(TIME_BASE + (reg))

void set_timer_interrupt(uint32_t timer, uint32_t delay_us) {
  if (timer == 1) {
    SYSTIME_REG(TIME_C1) = time_get() + delay_us;
  } else if (timer == 3) {
    SYSTIME_REG(TIME_C3) = time_get() + delay_us;
  }
}

void update_timer_interrupt(uint32_t timer, uint32_t delta_us) {
  if (timer == 1) {
    SYSTIME_REG(TIME_C1) = SYSTIME_REG(TIME_C1) + delta_us;
  } else if (timer == 3) {
    SYSTIME_REG(TIME_C3) = SYSTIME_REG(TIME_C3) + delta_us;
  }
  SYSTIME_REG(TIME_CS) = (1u << timer);
}

void clear_timer_interrupt(uint32_t timer) {
  SYSTIME_REG(TIME_CS) = (1u << timer);
}

uint32_t time_get() { return SYSTIME_REG(TIME_CLO); }
