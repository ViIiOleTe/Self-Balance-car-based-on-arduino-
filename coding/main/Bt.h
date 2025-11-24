#pragma once
#include "Config.h"

void btBegin();
bool btAvailable();
char btRead(); // read a bit
void btWrite(char c); // Send a bit
void btWriteString(const char* s);