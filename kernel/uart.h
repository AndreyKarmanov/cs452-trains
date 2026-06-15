#pragma once

#include <cstddef>

#define CONSOLE 0

void uart_config_and_enable(size_t line);

// Enum for UART interrupt identifiers
enum class UARTInterruptType {
  CTSMIM,
  RXIM,
  TXIM,
  RTIM,
};
void enable_uart_interrupt(UARTInterruptType interrupt_type);
void disable_uart_interrupt(UARTInterruptType interrupt_type);
void clear_uart_interrupt(UARTInterruptType interrupt_type);
bool is_uart_mis_rx_pending();
bool is_uart_mis_tx_pending();
bool is_uart_mis_cts_pending();
bool is_cts_clear_to_send();
bool can_receive_io();
bool can_transmit_io();
char getc();
void putc(char c);

// For debugging
char debug_getc(size_t line);
void debug_putc(size_t line, char c);
void debug_putl(size_t line, const char *buf, size_t blen);
void debug_puts(size_t line, const char *buf);
void debug_printf(size_t line, const char *fmt, ...);
