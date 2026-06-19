#include <cstdint>

#include "gic.h"
#include "rpi.h"

static char *const GIC_BASE  = reinterpret_cast<char *>(MMIO_BASE + 0x1840000);
static char *const GICD_BASE = GIC_BASE + 0x1000;
static char *const GICC_BASE = GIC_BASE + 0x2000;

#define GIC_REG(base, offset)                                                  \
  (*reinterpret_cast<volatile uint32_t *>((base) + (offset)))

static const uint32_t GICC_IAR             = 0x00C;
static const uint32_t GICC_EOIR            = 0x010;
static const uint32_t GICD_TARGETSRN_BASE  = 0x800;
static const uint32_t GICD_ISENABLERN_BASE = 0x100;
static const uint32_t GICD_ICENABLERN_BASE = 0x180;

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

void set_interrupt_core_routing(int core_id, int interrupt_id, bool enabled) {
  int n     = interrupt_id / 4;
  int shift = (interrupt_id % 4) * 8;
  int bit   = 1u << core_id;

  // GIC 4.3.12, GICD_ITARGETSRn
  if (enabled) {
    GIC_REG(GICD_BASE, GICD_TARGETSRN_BASE + (4 * n)) |= bit << shift;
  } else {
    GIC_REG(GICD_BASE, GICD_TARGETSRN_BASE + (4 * n)) &= ~(bit << shift);
  }
}

void set_interrupt(int interrupt_id, bool enabled) {
  auto n   = interrupt_id / 32;
  auto bit = 1u << (interrupt_id % 32);
  if (enabled) {
    // GIC 4.3.5, GICD_ISENABLERn
    GIC_REG(GICD_BASE, GICD_ISENABLERN_BASE + (4 * n)) |= bit;
  } else {
    // GIC 4.3.6, GICD_ICENABLERn
    GIC_REG(GICD_BASE, GICD_ICENABLERN_BASE + (4 * n)) |= bit;
  }
}
