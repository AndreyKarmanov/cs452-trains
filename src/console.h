#ifndef _console_h_
#define _console_h_ 1

#include "state.h"

typedef enum COMMAND_T {
	COMMAND_NONE,
    COMMAND_MRK,
	COMMAND_QUIT
} COMMAND_T;

void clear_console();
COMMAND_T update_console(State& state);

#endif /* _console_h_ */