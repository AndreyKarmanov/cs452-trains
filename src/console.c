#include "console.h"


// ok so basically, what we want is a ring buffer of these bytes
// we want a pop, push, etc. 
// if we get a "enter" then we pass the buffer into a "execute" engine that does things
// if it's nothing, then we just clear it
// if it is a command, then we want to exit. 
// handling this in the loop is something that