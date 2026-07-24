#pragma once

#include <cstdint>

static char *const MMIO_BASE = reinterpret_cast<char *>(0xFE000000);
static char *const GPIO_BASE = reinterpret_cast<char *>(MMIO_BASE + 0x200000);

void gpio_init();

uint32_t gpio_get_event_detect_status(uint32_t pin);
void gpio_clr_event_detect_status(uint32_t pin);
void gpio_init_interrupt();
