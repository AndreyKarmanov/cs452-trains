#include <cstdint>

#include "gic.h"
#include "rpi.h"

static char *const GIC_BASE  = (char *)(MMIO_BASE + 0x1840000);
static char *const GICD_BASE = GIC_BASE + 0x1000;
static char *const GICC_BASE = GIC_BASE + 0x2000;

#define GIC_REG(base, offset) (*(volatile uint32_t *)((base) + (offset)))

static const uint32_t GICC_IAR  = 0x00C;
static const uint32_t GICC_EOIR = 0x010;

uint32_t gic_iar_read() {
  // for multiprocessor implementations, this also returns the cpu id and needs
  // mask.
  // see gic doc sec. 4.4.4 for breakdown.
  return GIC_REG(GICC_BASE, GICC_IAR);
}

void gic_eoi(uint32_t gic_iar) {
  if ((gic_iar & GIC_IAR_ID_MASK) == GIC_SPURIOUS_IRQ) {
    return;
  }
  GIC_REG(GICC_BASE, GICC_EOIR) = gic_iar;
}
