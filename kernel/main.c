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
	size_t stack_size;
};

// This can live in the data section alongside other kernel data
TaskDescriptor task_descriptors[TASK_DESCRIPTORS];

// Make sure this lives in a separate, non-kernel section
uint8_t task_stacks[TASK_DESCRIPTORS][TASK_STACK_SIZE] __attribute__((section(".task_stacks")));

int _create(int priority, void (*function)()) {
	// kernel side handler of the create systemcall
	// finds an empty task descriptor, fills with appropriate values
	// and returns the tid of the created task
	return 0;
}

int _activate(int tid) {
	// this will trap to the kernel and the kernel will perform a context switch to the task with the given tid
	// when the task yields or makes a syscall, it will trap back to the kernel and return a request code that the task is making to the kernel (syscalls)
	return 0;
}

int _handle(int tid, int request) {
	// this will handle the given request code and perform the appropriate action (e.g. for syscalls)
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
	_create(0, shell); // not quite right

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
