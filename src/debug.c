#include "debug.h"
#include "uart.h"
#include <variant>

static char debug_to_hex_digit(uint8_t x) {
    return x < 10 ? (char)('0' + x) : (char)('A' + (x - 10));
}

static void debug_put_hex8(size_t line, uint8_t value) {
    uart_putc(line, debug_to_hex_digit((uint8_t)((value >> 4) & 0x0F)));
    uart_putc(line, debug_to_hex_digit((uint8_t)(value & 0x0F)));
}

void debug_put_bin8(size_t line, uint8_t value) {
    for (int group = 0; group < 2; ++group) {
        for (int bit = 3; bit >= 0; --bit) {
            uart_putc(line, (value & (1u << (bit + group * 8))) ? '1' : '0');
        }
        uart_putc(line, ' ');
    }
}

void debug_put_bin32(size_t line, uint32_t value) {
    for (int group = 0; group < 8; ++group) {
        for (int bit = 3; bit >= 0; --bit) {
            uart_putc(line, (value & (1u << (bit + group * 8))) ? '1' : '0');
        }
        uart_putc(line, ' ');
    }
}

void debug_clear_console(void) {
    uart_puts(CONSOLE, "\033[3;1H\033[K");
}

void debug_print_memory_dump(const void* start, uint32_t nbytes) {
    const uint8_t* bytes = (const uint8_t*)start;

    uart_puts(CONSOLE, "Memory dump\n\r");
    for (uint32_t row = 0; row < nbytes; row += 16) {
        uart_printf(CONSOLE, "%x: ", (uintptr_t)(bytes + row));
        for (uint32_t i = 0; i < 16; ++i) {
            if (row + i < nbytes) {
                debug_put_hex8(CONSOLE, bytes[row + i]);
            } else {
                uart_puts(CONSOLE, "  ");
            }
        }
        uart_putc(CONSOLE, '\n');
        uart_putc(CONSOLE, '\r');
    }
}

void debug_print_memory_bits(const void* start, uint32_t nbytes) {
    const uint8_t* bytes = (const uint8_t*)start;

    uart_puts(CONSOLE, "addr.  bits       hex\n\r");
    uart_puts(CONSOLE, "-----  ---------  ----\n\r");
    for (uint32_t i = 0; i < nbytes; ++i) {
        uart_printf(CONSOLE, "%x  ", (uintptr_t)(bytes + i));
        debug_put_bin8(CONSOLE, bytes[i]);
        uart_printf(CONSOLE, " 0x%x\n\r", bytes[i]);
    }
}

void debug_print_can_frame(const CANFRAME* frame) {
    uart_printf(
        CONSOLE,
        "CAN prio=%u cmd=0x%x resp=%u hash=0x%x dlc=%u data=[%x %x %x %x %x %x %x %x]\n\r",
        frame->prio,
        frame->cmdid,
        frame->resp,
        frame->hash,
        frame->dlc,
        frame->data[0],
        frame->data[1],
        frame->data[2],
        frame->data[3],
        frame->data[4],
        frame->data[5],
        frame->data[6],
        frame->data[7]
    );
}

void debug_print_mrk(const MRK_CMD& cmd) {
    switch (cmd.index()) {
    case 0: {
        const LightCommand& command = std::get<0>(cmd);
        uart_printf(CONSOLE, "LIGHT loco=%u value=%u\n\r", command.loco_id, command.value ? 1 : 0);
        return;
    }
    case 1: {
        const SpeedCommand& command = std::get<1>(cmd);
        uart_printf(CONSOLE, "SPEED loco=%u speed=%u\n\r", command.loco_id, command.speed);
        return;
    }
    case 2: {
        const DirectionCommand& command = std::get<2>(cmd);
        uart_printf(CONSOLE, "DIR loco=%u backward=%u\n\r", command.loco_id, command.backward ? 1 : 0);
        return;
    }
    case 3: {
        const SwitchCommand& command = std::get<3>(cmd);
        uart_printf(CONSOLE, "SWITCH sw=%u straight=%u\n\r", command.sw_id, command.straight ? 1 : 0);
        return;
    }
    case 4: {
        const SensorData& command = std::get<4>(cmd);
        uart_printf(
            CONSOLE,
            "SENSOR id=%u bank=%u number=%u old=%u new=%u\n\r",
            command.sensor_id,
            command.bank,
            command.number,
            command.old_state ? 1 : 0,
            command.new_state ? 1 : 0
        );
        return;
    }
    default: {
        const UnknownCommand& command = std::get<5>(cmd);
        uart_printf(CONSOLE, "UNKNOWN cmd=0x%x dlc=%u\n\r", command.frame.cmdid, command.frame.dlc);
        return;
    }
    }
}