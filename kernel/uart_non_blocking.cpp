#include <cstdarg>
#include <cstdint>

#include "rpi.h"
#include "uart.h"
#include "uart_non_blocking.h"
#include "util.h"

static char *const UART_BASE = (char *)(MMIO_BASE + 0x201000);
#define UART_REG(line, offset)                                                 \
  (*(volatile uint32_t *)(UART_BASE + line * 0x200 + offset))

// UART register offsets
static const uint32_t UART_DR   = 0x00;
static const uint32_t UART_FR   = 0x18;
static const uint32_t UART_IBRD = 0x24;
static const uint32_t UART_FBRD = 0x28;
static const uint32_t UART_LCRH = 0x2c;
static const uint32_t UART_CR   = 0x30;

// masks for specific fields in the UART registers
static const uint32_t UART_FR_BUSY = 0x08;
static const uint32_t UART_FR_RXFE = 0x10;
static const uint32_t UART_FR_TXFF = 0x20;
static const uint32_t UART_FR_RXFF = 0x40;
static const uint32_t UART_FR_TXFE = 0x80;

static const uint32_t UART_CR_UARTEN = 0x01;
static const uint32_t UART_CR_LBE    = 0x80;
static const uint32_t UART_CR_TXE    = 0x100;
static const uint32_t UART_CR_RXE    = 0x200;
static const uint32_t UART_CR_RTS    = 0x800;
static const uint32_t UART_CR_RTSEN  = 0x4000;
static const uint32_t UART_CR_CTSEN  = 0x8000;

static const uint32_t UART_LCRH_PEN       = 0x02;
static const uint32_t UART_LCRH_EPS       = 0x04;
static const uint32_t UART_LCRH_STP2      = 0x08;
static const uint32_t UART_LCRH_FEN       = 0x10;
static const uint32_t UART_LCRH_WLEN_LOW  = 0x20;
static const uint32_t UART_LCRH_WLEN_HIGH = 0x40;

// Configure the line properties (e.g, parity, baud rate) of a UART and ensure
// that it is enabled
void uart_config_and_enable(size_t line) {
  uint32_t baud_ival, baud_fval;
  uint32_t flag = UART_LCRH_FEN;

  switch (line) {
  // setting baudrate to approx. 115246.09844 (best we can do); 1 stop bit
  case CONSOLE:
    baud_ival = 26;
    baud_fval = 2;
    break;
  default:
    return;
  }

  // line control registers should not be changed while the UART is enabled, so
  // disable it
  uint32_t cr_state       = UART_REG(line, UART_CR);
  UART_REG(line, UART_CR) = cr_state & ~UART_CR_UARTEN;

  // set the baud rate
  UART_REG(line, UART_IBRD) = baud_ival;
  UART_REG(line, UART_FBRD) = baud_fval;

  // set the line control registers: 8 bit, no parity, 1 or 2 stop bits, FIFOs
  // enabled
  UART_REG(line, UART_LCRH) = UART_LCRH_WLEN_HIGH | UART_LCRH_WLEN_LOW | flag;

  // re-enable the UART; enable both transmit and receive regardless of previous
  // state
  UART_REG(line, UART_CR) =
      cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
}

char UARTNB::getc() { return UART_REG(line, UART_DR); }

bool UARTNB::can_receive_io() {
  return !(UART_REG(line, UART_FR) & UART_FR_RXFE);
}

void UARTNB::putc(char c) { tx_buffer.push(c); }

void UARTNB::putl(const char *buf, size_t blen) {
  for (size_t i = 0; i < blen; i++) {
    tx_buffer.push(buf[i]);
  }
}

void UARTNB::puts(const char *buf) {
  while (*buf) {
    tx_buffer.push(*buf);
    buf++;
  }
}

void UARTNB::printf(const char *fmt, ...) {
  va_list va;
  char ch, buf[12];

  va_start(va, fmt);
  while ((ch = *(fmt++))) {
    if (ch != '%')
      putc(ch);
    else {
      ch = *(fmt++);
      switch (ch) {
      case 'u':
        ui2a(va_arg(va, unsigned int), 10, buf);
        puts(buf);
        break;
      case 'd':
        i2a(va_arg(va, int), buf);
        puts(buf);
        break;
      case 'x':
        ui2a(va_arg(va, unsigned int), 16, buf);
        puts(buf);
        break;
      case 's':
        puts(va_arg(va, char *));
        break;
      case 'c':
        putc(va_arg(va, int));
        break;
      case '%':
        putc(ch);
        break;
      case '\0':
        return;
      }
    }
  }
  va_end(va);
}

void UARTNB::send_io() {
  if (tx_buffer.is_empty()) {
    return;
  }

  if (!(UART_REG(line, UART_FR) & UART_FR_TXFF)) {
    auto c = tx_buffer.pop();
    if (c.has_value()) {
      UART_REG(line, UART_DR) = c.value();
    }
  }
}
