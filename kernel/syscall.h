#pragma once

int create(int priority, void (*function)());
int my_tid();
int my_parent_tid();
void yield();
void exit();
