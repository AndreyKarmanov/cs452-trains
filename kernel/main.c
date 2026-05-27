#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "rpi.h"
#include "uart.h"
#include "shell.h"

#define TASK_STACK_SIZE 4096
#define TASK_DESCRIPTORS 4

extern "C" void setup_mmu(); // in mmu.S

// everything that we need to save to account for context-switch
struct TrapFrame
{
	// need to save x0 to x30 to allow for context switching
	uint64_t x[31];

	// also have to save the SP_EL0 and ELR_EL1 registers to allow for context switching
	uint64_t sp_el0; // this is the stack pointer
	uint64_t elr_el1; // this is where to jump after kernel code is done
	uint64_t spsr_el1; // Program stat, including condition codes, interrupt flags, execution state, etc. 
};

enum class TaskState
{
	READY,
	RUNNING,
	TERMINATED
};

struct TaskDescriptor
{
	int tid;
	int parent_tid;
	int priority;
	TaskState state;
	TrapFrame tf;
	uint8_t* stack_base;
};

// This can live in the data section alongside other kernel data
TaskDescriptor task_descriptors[TASK_DESCRIPTORS];

// Make sure this lives in a separate, non-kernel section
uint8_t task_stacks[TASK_DESCRIPTORS][TASK_STACK_SIZE] __attribute__((section(".task_stacks")));

int _create(int priority, void (*function)()) {
	// kernel side handler of the create systemcall
	// finds an empty task descriptor, fills with appropriate values
	// and returns the tid of the created task
	// write via asm - positional argument %0
	TaskDescriptor& td = task_descriptors[0];
	task_stacks[0][TASK_STACK_SIZE - 1] = 0;
	td.tid = 0;
	td.parent_tid = -1; // no parent
	td.priority = priority;
	td.state = TaskState::READY;

	td.stack_base = &task_stacks[0][TASK_STACK_SIZE - 1];
	td.tf.sp_el0 = (uint64_t)td.stack_base;
	td.tf.elr_el1 = (uint64_t)function;
	td.tf.spsr_el1 = 0;

	__builtin_memset(&td.tf.x, 0, sizeof(td.tf.x)); // zero registers

	return 0;
}

int _save(int tid) {
	TaskDescriptor& td = task_descriptors[tid];
	asm volatile("mrs %0, sp_el0" : "=r"(td.tf.sp_el0));
	asm volatile("mrs %0, elr_el1" : "=r"(td.tf.elr_el1));
	asm volatile("mrs %0, spsr_el1" : "=r"(td.tf.spsr_el1));

	// save x0 to x30
	asm volatile(
		"mov %0, x0\n\t"
		"mov %1, x1\n\t"
		"mov %2, x2\n\t"
		"mov %3, x3\n\t"
		"mov %4, x4\n\t"
		"mov %5, x5\n\t"
		"mov %6, x6\n\t"
		"mov %7, x7\n\t"
		"mov %8, x8\n\t"
		"mov %9, x9\n\t"
		"mov %10, x10\n\t"
		"mov %11, x11\n\t"
		"mov %12, x12\n\t"
		"mov %13, x13\n\t"
		"mov %14, x14\n\t"
		: "=r"(td.tf.x[0]), "=r"(td.tf.x[1]), "=r"(td.tf.x[2]), "=r"(td.tf.x[3]),
		"=r"(td.tf.x[4]), "=r"(td.tf.x[5]), "=r"(td.tf.x[6]), "=r"(td.tf.x[7]),
		"=r"(td.tf.x[8]), "=r"(td.tf.x[9]), "=r"(td.tf.x[10]), "=r"(td.tf.x[11]),
		"=r"(td.tf.x[12]), "=r"(td.tf.x[13]), "=r"(td.tf.x[14]));

	asm volatile(
		"mov %0, x15\n\t"
		"mov %1, x16\n\t"
		"mov %2, x17\n\t"
		"mov %3, x18\n\t"
		"mov %4, x19\n\t"
		"mov %5, x20\n\t"
		"mov %6, x21\n\t"
		"mov %7, x22\n\t"
		"mov %8, x23\n\t"
		"mov %9, x24\n\t"
		"mov %0, x25\n\t"
		"mov %1, x26\n\t"
		"mov %2, x27\n\t"
		"mov %3, x28\n\t"
		"mov %4, x29\n\t"
		"mov %5, x30\n\t"
		: "=r"(td.tf.x[15]), "=r"(td.tf.x[16]), "=r"(td.tf.x[17]), "=r"(td.tf.x[18]),
		"=r"(td.tf.x[19]), "=r"(td.tf.x[20]), "=r"(td.tf.x[21]), "=r"(td.tf.x[22]),
		"=r"(td.tf.x[23]), "=r"(td.tf.x[24]), "=r"(td.tf.x[25]), "=r"(td.tf.x[26]),
		"=r"(td.tf.x[27]), "=r"(td.tf.x[28]), "=r"(td.tf.x[29]), "=r"(td.tf.x[30]));
	return 0;
}

int _activate(int tid) {
	// this will trap to the kernel and the kernel will perform a context switch to the task with the given tid
	// when the task yields or makes a syscall, it will trap back to the kernel and return a request code that the task is making to the kernel (syscalls)
	TaskDescriptor& td = task_descriptors[tid];
	td.state = TaskState::RUNNING;

	// load registers
	asm volatile(
		"mov x0, %0\n\t"
		"mov x1, %1\n\t"
		"mov x2, %2\n\t"
		"mov x3, %3\n\t"
		"mov x4, %4\n\t"
		"mov x5, %5\n\t"
		"mov x6, %6\n\t"
		"mov x7, %7\n\t"
		"mov x8, %8\n\t"
		"mov x9, %9\n\t"
		"mov x10, %10\n\t"
		"mov x11, %11\n\t"
		"mov x12, %12\n\t"
		"mov x13, %13\n\t"
		"mov x14, %14\n\t" :: "r"(td.tf.x[0]), "r"(td.tf.x[1]), "r"(td.tf.x[2]), "r"(td.tf.x[3]),
		"r"(td.tf.x[4]), "r"(td.tf.x[5]), "r"(td.tf.x[6]), "r"(td.tf.x[7]),
		"r"(td.tf.x[8]), "r"(td.tf.x[9]), "r"(td.tf.x[10]), "r"(td.tf.x[11]),
		"r"(td.tf.x[12]), "r"(td.tf.x[13]), "r"(td.tf.x[14]));

	asm volatile(
		"mov x15, %0\n\t"
		"mov x16, %1\n\t"
		"mov x17, %2\n\t"
		"mov x18, %3\n\t"
		"mov x19, %4\n\t"
		"mov x20, %5\n\t"
		"mov x21, %6\n\t"
		"mov x22, %7\n\t"
		"mov x23, %8\n\t"
		"mov x24, %9\n\t"
		"mov x25, %10\n\t"
		"mov x26, %11\n\t"
		"mov x27, %12\n\t"
		"mov x28, %13\n\t"
		"mov x29, %14\n\t"
		"mov x30, %15\n\t"
		::  "r"(td.tf.x[15]),
		"r"(td.tf.x[16]), "r"(td.tf.x[17]), "r"(td.tf.x[18]), "r"(td.tf.x[19]),
		"r"(td.tf.x[20]), "r"(td.tf.x[21]), "r"(td.tf.x[22]), "r"(td.tf.x[23]),
		"r"(td.tf.x[24]), "r"(td.tf.x[25]), "r"(td.tf.x[26]), "r"(td.tf.x[27]),
		"r"(td.tf.x[28]), "r"(td.tf.x[29]), "r"(td.tf.x[30])
		);

	asm volatile("msr sp_el0, %0" :: "r"(td.tf.sp_el0));
	asm volatile("msr elr_el1, %0" :: "r"(td.tf.elr_el1));
	asm volatile("msr spsr_el1, %0" :: "r"(td.tf.spsr_el1));
	asm volatile("eret");

	return 0;
}

int _handle(int tid, int request) {
	// this will handle the given request code and perform the appropriate action (e.g. for syscalls)
	// ESR_EL1 will have exception code, holds n form svc N
	return 0;
}

extern "C" int kmain() {
#if defined(MMU)
	setup_mmu();
#endif
	gpio_init();
	uart_config_and_enable(CONSOLE);

	uart_puts(CONSOLE, "\033[2J\033[?25l\033[1;1H" __DATE__ " / " __TIME__ " / Andrey Karmanov / Anthony Ho\n\r");

	for (size_t i = 0; i < TASK_DESCRIPTORS; i++) {
		uart_printf(CONSOLE, "Task %u stack: 0x%x\n\r", i, &task_stacks[i]);
	}
	_create(0, shell);

	// for now we will have only 1 task
	for (;;) {
		int request = _activate(0);
		_handle(0, request);
	}

	return 0;
}

#if !defined(MMU)
#include <stddef.h>

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void* s, int c, size_t n) {
	for (char* it = (char*)s; n > 0; --n) *it++ = c;
	return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void* memcpy(void* dest, const void* src, size_t n) {
	char* sit = (char*)src;
	char* cdest = (char*)dest;
	for (size_t i = 0; i < n; ++i) *cdest++ = *sit++;
	return dest;
}
#endif
