#include "event_controller.h"
#include "gic.h"
#include "kernel_state.h"
#include "mcp2515.h"
#include "syscall.h"
#include "time.h"

void EventController::uninitialize_event(Event event) {
  initialized_events &= ~(1u << static_cast<int>(event));
}

void EventController::initialize_event(Event event) {
  // check if we've already initialized this event
  if (initialized_events & (1u << static_cast<int>(event)))
    return;
  initialized_events |= (1u << static_cast<int>(event));

  switch (event) {
  case Event::CLOCK_TICK: {
    set_interrupt_group0(GIC_TIMER_IRQ_C1, true);
    set_interrupt_core_routing(0, GIC_TIMER_IRQ_C1, true);
    set_interrupt(GIC_TIMER_IRQ_C1, true);
    clear_timer_interrupt(1);
    set_timer_interrupt(1, TICK_TIME_US);
    break;
  }
  case Event::DELAY_5S: {
    set_interrupt_group0(GIC_TIMER_IRQ_C3, true);
    set_interrupt_core_routing(0, GIC_TIMER_IRQ_C3, true);
    set_interrupt(GIC_TIMER_IRQ_C3, true);
    clear_timer_interrupt(3);
    set_timer_interrupt(3, TIME_1S_US * 5);
    break;
  }
  case Event::UART_RX_IRQ: {
    // unmask rtim and rxim
    enable_uart_interrupt(UARTInterruptType::RTIM);
    enable_uart_interrupt(UARTInterruptType::RXIM);
    break;
  }
  case Event::UART_TX_IRQ: {
    enable_uart_interrupt(UARTInterruptType::TXIM);
    break;
  }
  case Event::UART3_TX_IRQ: {
    enable_uart_interrupt(UARTInterruptType::TXIM, WEBSERIAL);
    break;
  }
  case Event::CAN_RX_IRQ: {
    auto active = mcp2515_get_active_irq();
    if (active.rxi0ie || active.rxi1e) {
      // short circuit, handle them if they're already active
      handle_event(Event::CAN_RX_IRQ);
    } else {
      // set up interrupts
      set_interrupt_group0(GIC_MCP2515_IRQ, true);
      set_interrupt_core_routing(0, GIC_MCP2515_IRQ, true);
      set_interrupt(GIC_MCP2515_IRQ, true);
      enable_mcp2515_interrupt(CANINT{.rxi1e = true, .rxi0ie = true});
    }
    break;
  }
  case Event::CAN_TX_IRQ: {
    auto active = mcp2515_get_active_irq();
    if (active.tx0ie || active.tx1ie || active.tx2ie || mcp2515_tx_ready()) {
      // short circuit, handle them if they're already active
      handle_event(Event::CAN_TX_IRQ);
    } else {
      // set up interrupts
      set_interrupt_group0(GIC_MCP2515_IRQ, true);
      set_interrupt_core_routing(0, GIC_MCP2515_IRQ, true);
      set_interrupt(GIC_MCP2515_IRQ, true);
      enable_mcp2515_interrupt(CANINT{
          .tx2ie = true,
          .tx1ie = true,
          .tx0ie = true,
      });
    }
    break;
  }
  default: {
    break;
  }
  }
}

void EventController::await_event(Event event, TaskId tid) {
  if (!event_buffers.contains(event)) {
    event_buffers.set(event, {});
  }
  auto event_buf = event_buffers.get_ref(event);
  event_buf->push(tid);
  initialize_event(event);
}

void EventController::handle_event(Event event, int arg0) {
  // one-time handling
  switch (event) {
  case Event::CLOCK_TICK: {
    update_timer_interrupt(1, TICK_TIME_US);
    clear_timer_interrupt(1);
    break;
  }
  case Event::DELAY_5S: {
    clear_timer_interrupt(3);
    uninitialize_event(event);
    break;
  }
  case Event::UART_RX_IRQ: {
    // mask so no RX IRQ fires until notifier re-await_event
    disable_uart_interrupt(UARTInterruptType::RXIM);
    disable_uart_interrupt(UARTInterruptType::RTIM);
    uninitialize_event(event);
    break;
  }
  case Event::UART_TX_IRQ: {
    // immediately disable after firing as they will keep firing
    disable_uart_interrupt(UARTInterruptType::TXIM);
    uninitialize_event(event);
    break;
  }
  case Event::UART3_TX_IRQ: {
    disable_uart_interrupt(UARTInterruptType::TXIM, WEBSERIAL);
    uninitialize_event(event);
    break;
  }
  case Event::CAN_RX_IRQ: {
    uninitialize_event(event);
    break;
  }
  case Event::CAN_TX_IRQ: {
    uninitialize_event(event);
    break;
  }
  default: {
    break;
  }
  }

  if (!event_buffers.contains(event)) {
    debug_printf(CONSOLE, "No waiting tasks for %d\n\r",
                 static_cast<int>(event));
    return;
  }

  // handle buffer of waiting tasks
  // some events may require all tasks to wake
  // some events may require the first to wake
  auto event_buf = event_buffers.get_ref(event);
  auto tid_opt   = event_buf->pop();
  while (tid_opt.has_value()) {
    auto tid = tid_opt.value();
    auto td  = kernel_runtime.task_table.lookup_td(tid);
    if (!td.has_value()) {
      _assert(false, "invalid tid in event buffer");
      tid_opt = event_buf->pop();
      continue;
    }

    auto *task = td.value();

    switch (event) {
    case Event::DELAY_5S: {
      kernel_runtime.scheduler.schedule(*task);

      // reset the delay for the next task.
      if (!event_buf->empty()) {
        initialize_event(event);
      }
      return; // return if only the first should wake
    }
    case Event::CAN_RX_IRQ: {
      kernel_runtime.scheduler.schedule(*task);
      return;
    }
    case Event::TASK_EXIT: {
      ((TrapFrame *)(task->sp_el0))->x[0] = arg0;
      kernel_runtime.scheduler.schedule(*task);
      break;
    }
    default: {
      kernel_runtime.scheduler.schedule(*task);
      break;
    }
    }

    tid_opt = event_buf->pop();
  }
}