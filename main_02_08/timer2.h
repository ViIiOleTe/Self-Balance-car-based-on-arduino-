#ifndef TIMER2_H
#define TIMER2_H

#include <Arduino.h>

// 5ms 控制标志（给 main 用）
extern volatile bool g_control5ms;

// 5ms tick 计数（可选：做40ms/500ms等）
extern volatile uint32_t g_tick5ms;

// 初始化 Timer2（5ms 中断）
void timer2_init_5ms(void);

#endif
