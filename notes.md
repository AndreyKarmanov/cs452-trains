# CS452 Trains

## You should know (or learn)

- what is an elf file exactly
- how are the files linked, loaded, compiled?
- what is a free-standing environment, what is a hosted environment?

## ideas

- build your own debugging tools
- multicore
- hardware level memory protection?
- actual virtual memory instead of identity mapping
- set up CPP to work

# Assignments

## A0: Polling Loop

Implement a big polling loop that essentially just reads memory in a loop and updates the UI. Also needs to communicate on CAN with the controller.

# Lectures

## Lec1: Intro

### newlib

libc implementation, but need to fake some OS services or implement some underlying features for it to work.

## Lec2: Memory

Note: all memory diagrams are`0xFFF...0x0` top to bottom
Note: Virtual addresses are identity mapped to physical memories

### Elf: Extended Linking Format

| Name         | Note                                                                         |
|--------------|------------------------------------------------------------------------------|
| Text Section |                                                                              |
| rodata       | Data that compiler decided is readonly                                       |
| data         |                                                                              |
| BSS          | All data that is initalized to 0, efficiency thing to avoid 1M "set to zero" |
| Constructors | Only if using CPP                                                            |

### ELF to BIN

usually there is a ELF loader that translates the ELF to img, however we have to do that ourselves. In IOtest, we can see that

| IMG location  | Note                                    |
|---------------|-----------------------------------------|
| BSS           | May not be set to 0 in (luke)warm start |
| rodata + data | May hardware protect rodata             |
| text          |                                         |

To ensure that BSS is all set to zero, can pull from linker script to initalize the entire section to zero.

### Using CPP

Have to make sure that constructors are run first

### MMU setup

| Name    | Note                                                            | Size             |
|---------|-----------------------------------------------------------------|------------------|
| Devices | Where devices live, like timers                                 | `0xFFFFFFFF`     |
| Data    | RW (top 76 MB may be used by video core)                        | `0x3FFFFFFF`     |
| Code    | our IMG (RO) (inital stack starts at 512KB `0x80000` downwards) | `0x1FFFFF` (2MB) |
| base    |                                                                 | `0x0`            |

### Devices

Chip is in "Low Peripheral Mode" Devices are available `0x7E000000` but it is mapped to `0xFE000000`, offset needs to be added to this base address to find the right device.

#### Timers

ARM timer: Linked to CPU frequency, don't really need

System Timer: 1 MHz, offset of `0x3000`

| Register | Note         | offset |
|----------|--------------|--------|
| Cs       | Status       | `0x04` |
| CLo      | Low 32 bits  | `0x08` |
| CHi      | High 32 bits | `0x12` |
| C0-C3    | Interupts    | `0x16` |

#### GPIO

GPIO can be programmed, don't have to touch setup GPIO

#### UART (Universal Asyc Recievor Transmittor)

Standard uses `115200 Hz`, 0 parity, 1 stop bit. We use UART0, at offset `0x201000`

| Register  | Note                        | offset |
|-----------|-----------------------------|--------|
| Data (DR) | Write to send, read to read | `0x00` |
| Flag      |                             | `0x18` |

#### SPI (Serial Peripheral Interface)

SPI is another mechanism similar to GPIO. Internal interface between the CPU and the CAN bus device. Defined in chapter 9 of broadcom manual. We use SPI0 at offset `0x204000`. Crank a lever to exchange bytes between eachother. Have to crank the lever multiple times to see out.
