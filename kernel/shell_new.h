#pragma once

void console_putc(int tx_tid, char c);
void console_puts(int tx_tid, const char *buf);
void console_printf(int tx_tid, const char *fmt, ...);

void shell_new_task();
