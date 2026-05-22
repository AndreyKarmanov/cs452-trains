#ifndef _console_h_
#define _console_h_ 1


typedef enum COMMAND_T {
	COMMAND_NONE,
    COMMAND_MRK,
	COMMAND_QUIT
} COMMAND_T;

COMMAND_T update_console();

#endif /* _console_h_ */