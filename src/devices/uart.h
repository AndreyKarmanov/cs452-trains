#pragma once

#include <cstddef>
#include <cstdint>

#define CONSOLE 0
#define WEBSERIAL 3

// PACTL_CS
static constexpr uint32_t PACTL_CS_OFFSET = 0x204E00;
static constexpr uint32_t PACTL_UART0_IRQ = 1u << 20;
static constexpr uint32_t PACTL_UART3_IRQ = 1u << 18;

void uart_config_and_enable(size_t line);

// Enum for UART interrupt identifiers
enum class UARTInterruptType {
  CTSMIM,
  RXIM,
  TXIM,
  RTIM,
};

uint32_t read_pactl_cs();

void enable_uart_interrupt(UARTInterruptType interrupt_type,
                           size_t line = CONSOLE);
void disable_uart_interrupt(UARTInterruptType interrupt_type,
                            size_t line = CONSOLE);
void clear_uart_interrupt(UARTInterruptType interrupt_type,
                          size_t line = CONSOLE);
bool is_uart_mis_rx_pending(size_t line = CONSOLE);
bool is_uart_mis_tx_pending(size_t line = CONSOLE);
bool is_uart_mis_cts_pending(size_t line = CONSOLE);
bool is_cts_clear_to_send(size_t line = CONSOLE);
bool can_receive_io(size_t line = CONSOLE);
bool can_transmit_io(size_t line = CONSOLE);
char getc(size_t line = CONSOLE);
void putc(char c, size_t line = CONSOLE);

// For debugging
char debug_getc(size_t line);
void debug_putc(size_t line, char c);
void debug_putl(size_t line, const char *buf, size_t blen);
void debug_puts(size_t line, const char *buf);
void debug_printf(size_t line, const char *fmt, ...);
