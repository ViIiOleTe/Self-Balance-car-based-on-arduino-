#ifndef TIMER2_H
#define TIMER2_H

#include <Arduino.h>

extern volatile bool g_control5ms;

extern volatile uint32_t g_tick5ms;

void timer2_init_5ms(void);

#endif
