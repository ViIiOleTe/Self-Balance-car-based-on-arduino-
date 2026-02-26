#include "timer2.h"
#include <avr/interrupt.h>

// ===== 全局变量定义 =====
volatile bool g_control5ms = false;
volatile uint32_t g_tick5ms = 0;


// ===== 初始化 Timer2：5ms =====
void timer2_init_5ms(void) {

  cli();   // 关总中断

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;

  // CTC 模式
  TCCR2A |= (1 << WGM21);

  // 16MHz / 1024 = 15625Hz
  // 5ms ≈ 78
  OCR2A = 78;

  // 1024 分频
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

  // 允许比较中断
  TIMSK2 |= (1 << OCIE2A);

  sei();   // 开总中断
}


// ===== Timer2 ISR =====
ISR(TIMER2_COMPA_vect) {
  g_tick5ms++;
  g_control5ms = true;
}
