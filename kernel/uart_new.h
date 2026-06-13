#pragma once

#include <cstddef>

#define CONSOLE 0

void uart_config_and_enable(size_t line);

// Enum for UART interrupt identifiers
enum class UARTInterruptType {
  RXIM,
  TXIM,
  RTIM,
};
void enable_uart_interrupt(UARTInterruptType interrupt_type);
void disable_uart_interrupt(UARTInterruptType interrupt_type);
void clear_uart_interrupt(UARTInterruptType interrupt_type);
bool is_uart_mis_rx_pending();
bool is_uart_mis_tx_pending();
bool can_receive_io();
bool can_transmit_io();
char getc();
void putc(char c);
