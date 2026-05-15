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
	.bss : {
		KEEP(*(.bss*))
	}
	__init__array_start = .;
	.init_array : {
		KEEP(*(.init_array*)) /* for C++ global constructors */
	}
	__init_array_end = .;
}
