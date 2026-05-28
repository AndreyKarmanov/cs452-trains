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

// https://developer.arm.com/documentation/102412/0103/Handling-exceptions/Taking-an-exception
.align 11 // align to 2^11 = 2 KiB
.global vectors
vectors:
	// Exception from the current EL while using SP_EL0
	// goes  sync / (fiq/vfiq) / (irq/virq) / (serror/vserror)
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	// Exception from the current EL while using SP_ELx
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	//Lower EL AArch64
	b _save_user_stack
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	//Lower EL AArch32
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124
	b default_handler
	.space 124

_save_user_stack:

	// switch to EL0 stack
	msr spsel, #0

	// make space for fake trap frame on EL0 stack
	// c++ loads structs at increasing address spaces
	// so we store registers in increasing order
	sub sp, sp, #272 

	// save user state to EL0 stack
	stp x0, x1, [sp, #0]
	stp x2, x3, [sp, #16]
	stp x4, x5, [sp, #32]
	stp x6, x7, [sp, #48]
	stp x8, x9, [sp, #64]
	stp x10, x11, [sp, #80]
	stp x12, x13, [sp, #96]
	stp x14, x15, [sp, #112]
	stp x16, x17, [sp, #128]
	stp x18, x19, [sp, #144]
	stp x20, x21, [sp, #160]
	stp x22, x23, [sp, #176]
	stp x24, x25, [sp, #192]
	stp x26, x27, [sp, #208]
	stp x28, x29, [sp, #224]

	mov x0, sp
	mrs x1, elr_el1
	mrs x2, spsr_el1

	stp x30, x0, [sp, #240]
	stp x1, x2, [sp, #256]

	// switch back to EL1 stack
	msr spsel, #1

	// load kernel state from el1 stack
	ldp x18, x19, [sp, #-160]
	ldp x20, x21, [sp, #-176]
	ldp x22, x23, [sp, #-192]
	ldp x24, x25, [sp, #-208]
	ldp x26, x27, [sp, #-224]
	ldp x28, x29, [sp, #-240]
	ldr x30, [sp, #-256]

	ret

.global _switch_to_user
_switch_to_user:
	// save kernel state
	stp x18, x19, [sp, #-160]
	stp x20, x21, [sp, #-176]
	stp x22, x23, [sp, #-192]
	stp x24, x25, [sp, #-208]
	stp x26, x27, [sp, #-224]
	stp x28, x29, [sp, #-240]
	str x30, [sp, #-256]

	// load user stack pointer into SP_EL0 from passed arg
	msr sp_el0, x0

	// switch to EL0 stack
	msr spsel, #0

	// restore user state from EL0 stack
	ldp x30, x0, [sp, #240] 
	ldp x1, x2, [sp, #256] // load elr_el1 and spsr_el1 from stack

	msr elr_el1, x1
	msr spsr_el1, x2

	ldp x0, x1, [sp, #0]
	ldp x2, x3, [sp, #16]
	ldp x4, x5, [sp, #32]
	ldp x6, x7, [sp, #48]
	ldp x8, x9, [sp, #64]
	ldp x10, x11, [sp, #80]
	ldp x12, x13, [sp, #96]
	ldp x14, x15, [sp, #112]
	ldp x16, x17, [sp, #128]
	ldp x18, x19, [sp, #144]
	ldp x20, x21, [sp, #160]
	ldp x22, x23, [sp, #176]
	ldp x24, x25, [sp, #192]
	ldp x26, x27, [sp, #208]
	ldp x28, x29, [sp, #224]

	// remove frame from top of stack
	add sp, sp, #272

	// switch back to EL1 stack
	msr spsel, #1

	// return to user code
	eret


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

	adr x0, vectors
	msr vbar_el1, x0
	isb

	// branch and link to main routine in C/C++
	bl bss_zero
	bl _init	
	bl kmain
	bl _fini

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
