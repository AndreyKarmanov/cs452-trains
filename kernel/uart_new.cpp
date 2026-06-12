// helper to enable and disable for rx and tx and rxtim
#include <cstdint>

#include "gic.h"
#include "rpi.h"
#include "uart.h"
#include "uart_non_blocking.h"

// we only use uart0
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

// interrupt
static const uint32_t UART_IFLS = 0x34;
static const uint32_t UART_IMSC = 0x38;
static const uint32_t UART_MIS  = 0x40;
static const uint32_t UART_ICR  = 0x44;

// IMSC: write 1 to unmask (enable), 0 to mask (disable)
static const uint32_t UART_IMSC_RXIM = 1 << 4;
static const uint32_t UART_IMSC_TXIM = 1 << 5;
static const uint32_t UART_IMSC_RTIM = 1 << 6;
static const uint32_t UART_IMSC_ALL  = 0x7FF;

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

  // setting RXIFLSEL (bits 5:3) and TXIFLSEL (bits 2:0) both to 1/2 full
  UART_REG(line, UART_IFLS) = (2 << 3) | 2;

  // mask all interrupt sources by default.
  UART_REG(line, UART_IMSC) = 0;

  // write to icr
  UART_REG(line, UART_ICR) = UART_IMSC_ALL;

  // re-enable the UART; enable both transmit and receive regardless of previous
  // state
  UART_REG(line, UART_CR) =
      cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;

  // config init interrupt states
  set_interrupt_core_routing(0, GIC_UART_IRQ, true);
  set_interrupt(GIC_UART_IRQ, true);
}

// void handle_uart_irq() {
//   // can use pactl_cs here to make sure that the interrupt is coming from
//   UART0
//   // but this is not necessary at this stage.
//   // static char *const PACTL_BASE = (char *)(MMIO_BASE + 0x204E00);

//   // read uart mis register to find out which uart interrupts
//   uint32_t mis = UART_REG(CONSOLE, UART_MIS);
//   if (mis & UART_IMSC_RXIM) {
//     // read uart dr register to get the received data
//     // uint32_t dr = UART_REG(CONSOLE, UART_DR);
//     // uart_putc(CONSOLE, dr);
//   }
//   if (mis & UART_IMSC_TXIM) {
//     // read uart dr register to get the transmitted data
//     // uint32_t dr = UART_REG(CONSOLE, UART_DR);
//     // uart_putc(CONSOLE, dr);
//   }
// }

// static void clock_notifier_task() {
//   int cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
//   _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

//   uart_printf(CONSOLE, "STARTED CLOCK NOTIFIER");

//   Message msg;
//   msg.type = MessageType::CS_TICK;
//   Message rcv_msg;

//   while (true) {
//     await_event(Event::CLOCK_TICK_1MS);
//     auto rcv_len = send(cs_tid, msg, rcv_msg);
//     _assert(rcv_len >= 0, "CLOCK TICK FAILED");
//   }
// }