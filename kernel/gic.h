#pragma once

#include <cstdint>

static constexpr uint32_t GIC_SPURIOUS_IRQ = 1023;
static constexpr uint32_t GIC_IAR_ID_MASK  = 0x3FF;
static constexpr uint32_t GIC_TIMER_IRQ_C1 = 97;
static constexpr uint32_t GIC_TIMER_IRQ_C3 = 99;

uint32_t gic_iar_read();
void gic_eoi(uint32_t irq);
