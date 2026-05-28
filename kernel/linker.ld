SECTIONS {
	. = 0x80000;            /* start text at this location */
	.text.boot : {          /* boot code must come first */
		KEEP(*(.text.boot))
	}
	.text : {               /* rest of code */
		KEEP(*(.text*))
	}
	. = 0x200000;           /* start data at this location for MMU setup */
	.rodata : {
		KEEP(*(.rodata*))
	}
	.data : {
		KEEP(*(.data*))
	}
	. = ALIGN(8); 				/* align bss to 8 bytes so we can quickly set in a loop */
	.bss : {
		__bss_start = .;
		KEEP(*(.bss*))
		. = ALIGN(8);			/* also align the end */
		__bss_end = .;
	}
	.init_array : {
		PROVIDE(__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		PROVIDE(__init_array_end = .);
	}

	.fini_array : {
		PROVIDE(__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array))
		PROVIDE(__fini_array_end = .);
	}
	.task_stacks : {
		__task_stacks_start = .;
		. = ALIGN(16);
		KEEP(*(.task_stacks))
		__task_stacks_end = .;
	}
}
