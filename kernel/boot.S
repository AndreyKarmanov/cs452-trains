// ***************************************
// SCTLR_EL1, System Control Register (EL1)
// Architecture Reference Manual Section D17.2.118
// ***************************************
#define SCTLR_RESERVED (3 << 28) | (3 << 22) | (1 << 20) | (1 << 11)
#define SCTLR_WFE_WFI  (1 << 18) | (1 << 16)

// ***************************************
// CPACR_EL1, Architectural Feature Access Control Register (EL1)
// Architecture Reference Manual Section D17.2.30
// CPTR_EL2, Architectural Feature Trap Register (EL2)
// Architecture Reference Manual Section D17.2.31
// ***************************************
#define FPEN_TRAP (3 << 20)
#define  ZEN_TRAP (3 << 16)

// ***************************************
// HCR_EL2, Hypervisor Configuration Register (EL2)
// Architecture Reference Manual Section D17.2.48
// ***************************************
#define HCR_RW (1 << 31)

// ***************************************
// SPSR_EL2, Saved Program Status Register (EL2)
// Architecture Reference Manual Section C5.2.19
// ***************************************
#define SPSR_MASK_ALL (11 << 6) // mask D,I,F
#define SPSR_EL1       (5 << 0) // EL1 with SP_EL1

// ensure the linker puts this at the start of the image
.section ".text.boot"
.global _start
_start:
	// if this is not Core 0, exit to wfe-loop
	mrs  x0, mpidr_el1
	and  x0, x0, #3
	cbnz x0, exit

	// enable SMP cores (needed for MMU, caching)
	mrs x0, s3_1_c15_c2_1
	orr x0, x0, #(0x1 << 6)
	msr s3_1_c15_c2_1, x0

	// are we already in EL1?
	// if so, we jump to el1_entry
	mrs	x1, CurrentEL
	and	x1, x1, #8
	cbz	x1, el1_entry

	// disable EL1 trap on FP/SIMD/SVE instructions
	ldr x2, =(FPEN_TRAP | ZEN_TRAP)
	msr cptr_el2, x2

	// set execution state for EL1 to AArch64
	ldr x3, =HCR_RW
	msr hcr_el2, x3

	// switch to EL1 by fake exception return
	ldr x4, =(SPSR_MASK_ALL | SPSR_EL1)
	msr spsr_el2, x4
	adr x5, el1_entry
	msr elr_el2, x5

	eret // -> el1_entry

el1_entry:
	// configure processor
	ldr x0, =(SCTLR_RESERVED | SCTLR_WFE_WFI)
	msr sctlr_el1, x0

	// disable EL0 trap on FP/SIMD/SVE instructions
	ldr x1, =(FPEN_TRAP | ZEN_TRAP)
	msr cpacr_el1, x1

	// initialize SP (initial stack below text here)
	ldr x2, =_start
	mov sp, x2

	// zero out bss
	ldr x19, =__bss_start
	ldr x20, =__bss_end
loop_bss_zero:
    cmp x19, x20          	// compare current with end
    b.ge done_bss_zero		// if current >= end, we're done
	str xzr, [x19]
	add x19, x19, #8
	b loop_bss_zero
done_bss_zero:

	// call each function in init array
	// we use x19-x28 for callee-saved registers, aka we don't have to worry about clobbering by convention (followed in c/cpp)
	// note: if I write hand-assembly in future, need to make sure i'm not clobbering these!
	ldr x19, =__init_array_start
	ldr x20, =__init_array_end
loop_init_array:
    cmp x19, x20          	// compare current with end
    b.ge done_init_array	// if current >= end, we're done
    ldr x2, [x19]        	// load function pointer at [x19]
    blr x2              	// call the function
    add x19, x19, #8      	// move to next 64-bit pointer 
    b loop_init_array
done_init_array:

	ldr x19, =loop_stack_zero
	ldr x20, =loop_stack_zero
loop_stack_zero:
	cmp x19, x20
	b.ge done_stack_zero
	str xzr, [x19]
	add x19, x19, #8
	b loop_stack_zero
done_stack_zero:

	// branch and link to main routine in C/C++
	bl kmain

.global _reboot
_reboot:
	// use watchdog to reboot
	mov x2, 36
	mov x0, 28
	movk	x2, 0xfe10, lsl 16
	movk	x0, 0xfe10, lsl 16
	mov w3, 1 
	mov w1, 32
	movk	w3, 0x5a00, lsl 16
	str w3, [x2]
	movk	w1, 0x5a00, lsl 16
	str w1, [x0]
	// shouldn't be reached

exit:
	wfe
	b exit
