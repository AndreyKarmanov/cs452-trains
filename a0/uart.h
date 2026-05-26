#ifndef _uart_h_
#define _uart_h_ 1

#include <stddef.h>
#include <stdint.h>

#define CONSOLE 0

void uart_config_and_enable(size_t line);
char uart_maybec(size_t line);
char uart_getc(size_t line);
void uart_putc(size_t line, char c);
void uart_putl(size_t line, const char *buf, size_t blen);
void uart_puts(size_t line, const char *buf);
void uart_printf(size_t line, const char *fmt, ...);
void uart_flush(size_t line, uint32_t budget_us);
void uart_flush_all(size_t line);
uint32_t uart_tx_dropped(size_t line);
void clear_uart_dropped(size_t line);

#endif /* uart.h */
