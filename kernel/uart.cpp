// helper to enable and disable for rx and tx and rxtim
#include "uart.h"
#include "gic.h"
#include "rpi.h"
#include "util.h"
#include <cstdarg>
#include <cstdint>

#define UART_BASE reinterpret_cast<char *>(MMIO_BASE + 0x201000)
#define UART_REG(line, offset)                                                 \
  (*reinterpret_cast<volatile uint32_t *>(UART_BASE + (line) * 0x200 +         \
                                          (offset)))

// UART register offsets
static const uint32_t UART_DR   = 0x00;
static const uint32_t UART_FR   = 0x18;
static const uint32_t UART_IBRD = 0x24;
static const uint32_t UART_FBRD = 0x28;
static const uint32_t UART_LCRH = 0x2c;
static const uint32_t UART_CR   = 0x30;

// masks for specific fields in the UART registers
static const uint32_t UART_FR_CTS  = 0x01;
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

// interrupt
static const uint32_t UART_IFLS = 0x34;
static const uint32_t UART_IMSC = 0x38;
static const uint32_t UART_MIS  = 0x40;
static const uint32_t UART_ICR  = 0x44;

// IMSC: write 1 to unmask (enable), 0 to mask (disable)
static const uint32_t UART_IMSC_CTSMIM = 1 << 1;
static const uint32_t UART_IMSC_RXIM   = 1 << 4;
static const uint32_t UART_IMSC_TXIM   = 1 << 5;
static const uint32_t UART_IMSC_RTIM   = 1 << 6;
static const uint32_t UART_IMSC_ALL    = 0x7FF;

uint32_t read_pactl_cs() {
  return *reinterpret_cast<volatile uint32_t *>(MMIO_BASE + PACTL_CS_OFFSET);
}

// Configure the line properties (e.g, parity, baud rate) of a UART and ensure
// that it is enabled
void uart_config_and_enable(size_t line) {
  uint32_t baud_ival, baud_fval;
  uint32_t flag = UART_LCRH_FEN;
  bool tx_only  = false;

  switch (line) {
  // setting baudrate to approx. 115246.09844 (best we can do); 1 stop bit
  case CONSOLE:
    baud_ival = 26;
    baud_fval = 2;
    break;
  case WEBSERIAL:
    baud_ival = 26;
    baud_fval = 2;
    tx_only   = true;
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

  // setting RXIFLSEL (bits 5:3) and TXIFLSEL (bits 2:0) both to 1/2 full
  UART_REG(line, UART_IFLS) = (2 << 3) | 2;

  // mask all interrupt sources by default.
  UART_REG(line, UART_IMSC) = 0;

  // write to icr
  UART_REG(line, UART_ICR) = UART_IMSC_ALL;

  // re-enable the UART
  if (tx_only) {
    UART_REG(line, UART_CR) = UART_CR_UARTEN | UART_CR_TXE;
  } else {
    UART_REG(line, UART_CR) =
        cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
  }

  // config init interrupt states (shared GIC 153 for all PL011s)
  set_interrupt_group0(GIC_UART_IRQ, true);
  set_interrupt_core_routing(0, GIC_UART_IRQ, true);
  set_interrupt(GIC_UART_IRQ, true);
}

void enable_uart_interrupt(UARTInterruptType interrupt_type, size_t line) {
  switch (interrupt_type) {
  case UARTInterruptType::CTSMIM:
    UART_REG(line, UART_IMSC) |= UART_IMSC_CTSMIM;
    break;
  case UARTInterruptType::RXIM:
    UART_REG(line, UART_IMSC) |= UART_IMSC_RXIM;
    break;
  case UARTInterruptType::TXIM:
    UART_REG(line, UART_IMSC) |= UART_IMSC_TXIM;
    break;
  case UARTInterruptType::RTIM:
    UART_REG(line, UART_IMSC) |= UART_IMSC_RTIM;
    break;
  default:
    break;
  }
}

void disable_uart_interrupt(UARTInterruptType interrupt_type, size_t line) {
  switch (interrupt_type) {
  case UARTInterruptType::CTSMIM:
    UART_REG(line, UART_IMSC) &= ~UART_IMSC_CTSMIM;
    break;
  case UARTInterruptType::RXIM:
    UART_REG(line, UART_IMSC) &= ~UART_IMSC_RXIM;
    break;
  case UARTInterruptType::TXIM:
    UART_REG(line, UART_IMSC) &= ~UART_IMSC_TXIM;
    break;
  case UARTInterruptType::RTIM:
    UART_REG(line, UART_IMSC) &= ~UART_IMSC_RTIM;
    break;
  default:
    break;
  }
}

void clear_uart_interrupt(UARTInterruptType interrupt_type, size_t line) {
  switch (interrupt_type) {
  case UARTInterruptType::CTSMIM:
    UART_REG(line, UART_ICR) = UART_IMSC_CTSMIM;
    break;
  case UARTInterruptType::RXIM:
    UART_REG(line, UART_ICR) = UART_IMSC_RXIM;
    break;
  case UARTInterruptType::TXIM:
    UART_REG(line, UART_ICR) = UART_IMSC_TXIM;
    break;
  case UARTInterruptType::RTIM:
    UART_REG(line, UART_ICR) = UART_IMSC_RTIM;
    break;
  default:
    break;
  }
}

bool is_uart_mis_rx_pending(size_t line) {
  bool rx_timer_pending = (UART_REG(line, UART_MIS) & UART_IMSC_RTIM) != 0;
  bool rx_interrupt_pending =
      (UART_REG(line, UART_MIS) & UART_IMSC_RXIM) != 0;
  return rx_timer_pending || rx_interrupt_pending;
}

bool is_uart_mis_tx_pending(size_t line) {
  return (UART_REG(line, UART_MIS) & UART_IMSC_TXIM) != 0;
}

bool is_uart_mis_cts_pending(size_t line) {
  return (UART_REG(line, UART_MIS) & UART_IMSC_CTSMIM) != 0;
}

bool is_cts_clear_to_send(size_t line) {
  return (UART_REG(line, UART_FR) & UART_FR_CTS) != 0;
}

bool can_receive_io(size_t line) {
  return !(UART_REG(line, UART_FR) & UART_FR_RXFE);
}

bool can_transmit_io(size_t line) {
  uint32_t fr = UART_REG(line, UART_FR);
  if (fr & UART_FR_TXFF) {
    return false;
  }
  return true;
}

char getc(size_t line) { return static_cast<char>(UART_REG(line, UART_DR)); }
void putc(char c, size_t line) { UART_REG(line, UART_DR) = c; }

// debug functions
char debug_getc(size_t line) {
  while (UART_REG(line, UART_FR) & UART_FR_RXFE)
    ; // wait for data ready
  return static_cast<char>(UART_REG(line, UART_DR));
}

// For debugging
void debug_putc(size_t line, char c) {
  while (UART_REG(line, UART_FR) & UART_FR_TXFF)
    ; // wait for room in buffer
  UART_REG(line, UART_DR) = c;
}

void debug_putl(size_t line, const char *buf, size_t blen) {
  for (size_t i = 0; i < blen; i++) {
    debug_putc(line, *(buf + i));
  }
}

void debug_puts(size_t line, const char *buf) {
  while (*buf) {
    debug_putc(line, *buf);
    buf++;
  }
}

void debug_printf(size_t line, const char *fmt, ...) {
  va_list va;
  char ch, buf[12];

  va_start(va, fmt);
  while ((ch = *(fmt++))) {
    if (ch != '%')
      debug_putc(line, ch);
    else {
      ch = *(fmt++);
      switch (ch) {
      case 'u':
        ui2a(va_arg(va, unsigned int), 10, buf);
        debug_puts(line, buf);
        break;
      case 'd':
        i2a(va_arg(va, int), buf);
        debug_puts(line, buf);
        break;
      case 'x':
        ui2a(va_arg(va, unsigned int), 16, buf);
        debug_puts(line, buf);
        break;
      case 's':
        debug_puts(line, va_arg(va, char *));
        break;
      case 'c':
        debug_putc(line, va_arg(va, int));
        break;
      case 'b':
        ui2a(va_arg(va, unsigned int), 2, buf);
        debug_puts(line, buf);
        break;
      case '%':
        debug_putc(line, ch);
        break;
      case '\0':
        return;
      }
    }
  }
  va_end(va);
}
