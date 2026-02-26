#include "timer2.h"
#include <avr/interrupt.h>

volatile bool g_control5ms = false;
volatile uint32_t g_tick5ms = 0;



void timer2_init_5ms(void) {

  cli();//turn off GI

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;

  // CTC(clear time on campare match)
  TCCR2A |= (1 << WGM21);

  // 16MHz / 1024 = 15625Hz
  // 5ms ≈ 78
  OCR2A = 78;

  // 1024 prescale
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

  // enable compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();
}


// ===== Timer2 ISR =====
ISR(TIMER2_COMPA_vect) {
  g_tick5ms++;
  g_control5ms = true;
}
