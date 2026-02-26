#include "Encoder.h"
#include <Arduino.h>
#include "Timer2.h"


static volatile long pulse_countA = 0;
static volatile long pulse_countB = 0;


static uint32_t lastSampleTick = 0;


static inline uint16_t ms_to_ticks(uint16_t ms) {

  uint16_t t = (ms + 4) / 5;
  return (t == 0) ? 1 : t;
}

void Encoder_init() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PMA, INPUT);
  pinMode(PMB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, RISING);

  noInterrupts();
  lastSampleTick = g_tick5ms;
  interrupts();
}

void readEncoderA() {
  int a = digitalRead(ENCA);
  int b = digitalRead(PMA);
  if (a == b) pulse_countA++;   // forward
  else        pulse_countA--;   // backward
}

void readEncoderB() {
  int a = digitalRead(ENCB);
  int b = digitalRead(PMB);
  if (a == b) pulse_countB++;   // forward
  else        pulse_countB--;   // backward
}

/**
 * @brief 固定时间窗采样：返回这段时间内左右轮脉冲数，并清零计数
 * @param pA 输出：A轮脉冲数（可正可负）
 * @param pB 输出：B轮脉冲数（可正可负）
 * @param window_ms 采样窗口（建议 40ms，或 20ms；最好是 5 的倍数）
 * @return 是否到达采样时间窗
 */
bool encoderSamplePulses(long& pA, long& pB, uint16_t window_ms) {
  uint16_t needTicks = ms_to_ticks(window_ms);

  uint32_t nowTick;
  noInterrupts();
  nowTick = g_tick5ms;
  interrupts();

  if ((uint32_t)(nowTick - lastSampleTick) < needTicks) return false;

  noInterrupts();
  pA = -pulse_countA;
  pB =  pulse_countB;
  pulse_countA = 0;
  pulse_countB = 0;
  interrupts();

  lastSampleTick = nowTick;
  return true;
}

/**
 * @brief 固定时间窗采样：直接返回左右脉冲和（速度环常用）
 */
bool encoderSampleSum(long& sum, uint16_t window_ms) {
  long a, b;
  if (!encoderSamplePulses(a, b, window_ms)) return false;
  sum = a + b;
  return true;
}

/**
 * @brief 仍保留 RPM 接口（可选）
 *        注意：RPM 用 tick 算 dt，window_ms 最好设为 10/20/40 这种
 */
bool encoderSampleRPM(float& rpmA, float& rpmB) {
  const uint16_t window_ms = 10;
  uint16_t needTicks = ms_to_ticks(window_ms);

  uint32_t nowTick;
  noInterrupts();
  nowTick = g_tick5ms;
  interrupts();

  if ((uint32_t)(nowTick - lastSampleTick) < needTicks) return false;

  long countA, countB;
  noInterrupts();
  countA = pulse_countA;
  countB = pulse_countB;
  pulse_countA = 0;
  pulse_countB = 0;
  interrupts();

  // dt 用 tick 精确得到（每 tick=5ms）
  float dt = (nowTick - lastSampleTick) * 0.005f; // seconds

  const float ticks_per_rev = pulses_per_rev * gear_ratio;
  rpmA = (countA / ticks_per_rev) / dt * 60.0f;
  rpmB = (countB / ticks_per_rev) / dt * 60.0f;

  lastSampleTick = nowTick;
  return true;
}

