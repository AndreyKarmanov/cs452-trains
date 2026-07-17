#pragma once

#include <cstdint>

static constexpr uint32_t GIC_SPURIOUS_IRQ = 1023;
static constexpr uint32_t GIC_IAR_ID_MASK  = 0x3FF;
static constexpr uint32_t GIC_TIMER_IRQ_C1 = 97;
static constexpr uint32_t GIC_TIMER_IRQ_C3 = 99;
static constexpr uint32_t GIC_UART_IRQ     = 153;
static constexpr uint32_t GIC_MCP2515_IRQ  = 145;

void gic_init();
uint32_t gic_iar_read();
void gic_eoi(uint32_t gic_iar);

void set_interrupt_group0(int interrupt_id, bool enabled);
void set_interrupt_core_routing(int core_id, int interrupt_id, bool enabled);
void set_interrupt(int interrupt_id, bool enabled);
