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
- set up CPP

# Documentation

## Getting C++ to work

Generally, C++ works almost out of the box. The main things that need changes is a) symbols used in `Boot.s` need to be decorated with `extern "C"`, e.g. the `kmain` function needs this so that `Boot.s` sees and it and can jump to it.

### Implemented

- global objects with non-trivial constructors
    - need to initalize by iterating through & jumping to the addresses in `.init_array`
- stub syscalls
    - https://sourceware.org/newlib/libc.html#Syscalls
    - bare minimum ones to support
    - after you use a lbrary (e.g. std::variant) you can try compiling and for each syscall that is missing during compilation, add a stub
    - I'm hoping this will be good enough, I have no idea (will prob get screwed later)

### Not implemented (yet?)

- global or function-local static objects with non-trivial destructors
    - need to iterate through `.fini_array`
    - support `__cxa_atexit` which is registering destructors, required for function-local static object destructors because they are lazy.
    - there's also a `__cxa_finalize` and `__dso_handle` thing but I don't really understand what those are for, think it's for running the destructors and some sort of ID, but I don't get it and don't really need it just yet so not bothering with implementation
- malloc / free / `new` / `delete`
    - these are required for a number of different functions. most notably for vectors and probably some OS-level stuff (threads?) but those can be handled with fixed size arrays for now.
- standard library support
    - haven't really looked into what is required, but I suspect It can be added incrementally

## A0: Polling Loop

Implement a big polling loop that essentially just reads memory in a loop and updates the UI. Also needs to communicate on CAN with the controller.

- initalizing variables to 0
    - .bss section must be zero'd out (worth aligning to 8 byte boundaries  with `ALIGN(8)` to make it easy)

tips & tricks: 
- think of how the memory is laid out in the mcp2515, can easily structure code to simplify reading and writing
- also make sure to read the mcp2515 documentation clearly, there are instructions that can simplify your work significantly (and they are likely not provided for you in iotest)

## K1: Context switching, syscalls
- again, think of how the memory is laid out. trapframes can be easily accessed with the right datastructure to remove any need for inline ASM 

## K2: Send, Receive, Reply, Nameserver, Caching
- Caching 
    - 

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


<!-- ../../qemu/build/qemu-system-aarch64 -M raspi4b -cpu cortex-a53 -m 2G -serial stdio -display none -kernel iotest.img -->

## lec8: interrupts + timers

As busy waiting is messy, power hungry 

edge-triggered: blip in signal, for event, fired once per change, e.g. new message arrived
level-triggered: interrupt is raised while some state is active, e.g. "BUFFERFULL"

### types of interrupts

- shared peripheral interrupt (SPI but not our SPI)
    - these are the ones we care about, all our devices are SPI
- Private peripheral interrupt (PPI) 
    - our RPI doens't have / we don't use
- Software generated interrupt (SGI)
    - cores can use this explicitly to alert other cores

At time of interrupt delivery, the processor will complete a pipeline flush, after an instruction. It will then find the handler in VBAR. We are looking still looking at the lower EL group 64 bit synchronous. Difficulty comes as we don't know where we are, as opposed to SVC for system calls which were glorified function calls. 

### FIQ vs IRQ:
- Fast interrupt is legacy mechanism from 32 bit. We don't use this, as it's legacy. It banked some registers for you, which made it faster as no state had to be saved in some simpler cases. 

interrupts in the cpu can be masked 
   - DAIF, these are 4 bits of the pstate.
   - if the I bit is set, then the interrupts will be ignored, when exception occurs, interrupts get masked automatically. We need to ensure the bit is 0 to have interrupts enabled for new user tasks.
   - spsr_el1 -> holds pstate of el0, set when an exception occurs, copied back into pstate
   - 
### interrupt controller

The GIC (generic interrupt controller) sits between the device & cpu. We have a GIC 400 (GIC v2) we do not use legacy interrupt controller. It's job is similar to a network switch. It can have multiple devices connected, and also can have multiple cpus wired up. It can schedule interrupts and also assign priorities to them. We can offload this from CPU to GIC, however prof recommends us to stay away from it, sufficient to let CPU deal with it in software. GIC also converts edge triggers into level triggers, which helps us catch any edge interrupts we may miss when interrupts are masked. 

GIC is split into two logical pieces: 
- GICD: GIC Distributor
    - set up at boottime
- GICC: CPU Interface
    - used during runtime to juggle interrupts
    - Manages CPU across two dimensions: 
        - (Pending / Not Pending) 
            - Level: is the device asserting? 
            - Edge: has the device asserted? 
        - (Active / Not Active)
            - is CPU handling the interrupt
        - States:
            - not pending & not active: inactive
            - not pending & active: active
            - pending & not active: pending
            - pending & active: pending & active
        - When handler is done, to remove Active state, it writes interrupt number to GICC_EOIR

At boot (iotest):
- GIC is enabled
- Interrupts are disabled
- Interrupt routing is undefined
- In EL1, interrupts are masked

In startup routine, we must program the interrupts we want. 

### Timer interrupts

- These are actually the first 4 VC (videocore) interrupts 
- C0, C2 are reserved
- C1, C3 are the ones we use
- Compute some time in the future, set the value in C1 / C3, the interrupt will be generated.
- C1 has InterruptId 97, and C2 has interrupt 99

### Routing Interrupts

- GICD_ITARGETSRn
    - Each register defines targets for 4 interrupt
    - 1024 interrupts, 8 bits = 8 cores
- Find the byte that corresponds to the interrupt we want, and then set the bit corresponding to the core we want. 
- To enable interrupt, use GICD_ISENABLERn
    - 4 byte registers, with 1 bit per interruptId
    - 1024 Interrupts, 1 bit.

### Handling

- Floating point state may have to be saved, can check if it's been used (there's some register or something)
- Difficult to avoid floating points entirely (compiler assumes you have) but we can avoid mostly (?) didn't entirely remember
- GICC_IAR holds the interrupt ID
    - sets interrupt state to Active in GIC
- Re-arm te interrupt via CS status register of the timer (may have to do at boot too)
- write to GICC_EOIR to let it konw we're done

### Optimizations
- An optimization is to include a loop to read the GICC_IRR again after handling it in the kernel, in case a second interrupt is pending. This saves the overhead of scheduling a task that gets immediately interrupted. 
- Can use a hybrid approach with polling, if there are lots of interrupts it is lots of overhead, so if we get an interrupt, we can try polling with interrupts disabled, and only when ~5 polls fail then we re-enable interrupts to go back to them. Useful in high frequency devices like network handlers, which can have bursts of work. 
