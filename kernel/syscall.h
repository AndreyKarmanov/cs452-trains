#pragma once

int Create(int priority, void (*function)());
int MyTid();
int MyParentTid();
void Yield();
void Exit();
