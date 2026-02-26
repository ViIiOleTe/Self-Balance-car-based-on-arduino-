#include "Encoder.h"
#include <Arduino.h>

// ISR 会修改，必须 volatile
static volatile long pulse_countA = 0;
static volatile long pulse_countB = 0;

// 采样计时
static unsigned long lastSampleMs = 0;

void Encoder_init() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PMA, INPUT);
  pinMode(PMB, INPUT);

  // 建议用 CHANGE 更细，但你原来用 RISING 也能用（分辨率更低）
  // attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, RISING);

  lastSampleMs = millis();
}

// 左轮（或电机A）编码器中断
void readEncoderA() {
  // 注意：你用 RISING 的话 a 永远是 HIGH（这一瞬间），其实不需要读 a
  // 方向判断靠另一相 b 就够了。为了保持你原逻辑，我保留写法。
  int a = digitalRead(ENCA);
  int b = digitalRead(PMA);
  if (a == b) pulse_countA++;   // forward
  else        pulse_countA--;   // backward
}

// 右轮（或电机B）编码器中断
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
 * @param window_ms 采样窗口（建议 40ms，或 20ms）
 * @return 是否到达采样时间窗
 */
bool encoderSamplePulses(long& pA, long& pB, uint16_t window_ms) {
  unsigned long now = millis();
  if ((uint16_t)(now - lastSampleMs) < window_ms) return false;

  noInterrupts();
  pA = -pulse_countA;
  pB = pulse_countB;
  pulse_countA = 0;
  pulse_countB = 0;
  interrupts();

  lastSampleMs = now;
  return true;
}

/**
 * @brief 固定时间窗采样：直接返回左右脉冲和（速度环常用）
 * @param sum 输出：pA + pB
 * @param window_ms 采样窗口（建议 40ms）
 * @return 是否到达采样时间窗
 */
bool encoderSampleSum(long& sum, uint16_t window_ms) {
  long a, b;
  if (!encoderSamplePulses(a, b, window_ms)) return false;
  sum = a + b;
  return true;
}

/**
 * @brief 仍保留 RPM 接口（可选），但平衡车速度环建议用脉冲数接口
 */
bool encoderSampleRPM(float& rpmA, float& rpmB) {
  const uint16_t window_ms = 10; // 你原来 10ms
  unsigned long now = millis();
  if ((uint16_t)(now - lastSampleMs) < window_ms) return false;

  long countA, countB;
  noInterrupts();
  countA = pulse_countA;
  countB = pulse_countB;
  pulse_countA = 0;
  pulse_countB = 0;
  interrupts();

  const float ticks_per_rev = pulses_per_rev * gear_ratio; // 输出轴一圈脉冲数
  float dt = (now - lastSampleMs) / 1000.0f;
  rpmA = (countA / ticks_per_rev) / dt * 60.0f;
  rpmB = (countB / ticks_per_rev) / dt * 60.0f;

  lastSampleMs = now;
  return true;
}

// #include "Encoder.h"
// #include <Arduino.h>

// // volatile 供中断修改
// static volatile long pulse_countA = 0;
// static volatile long pulse_countB = 0;

// static unsigned long lastTimeMs = 0;

// void Encoder_init(){
//   pinMode(ENCA, INPUT);
//   pinMode(ENCB, INPUT);
//   pinMode(PMA, INPUT);
//   pinMode(PMB, INPUT);
//   attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, RISING);
// }

// void readEncoderA() {
//   int a = digitalRead(ENCA);
//   int b = digitalRead(PMA);
//   if (a == b) pulse_countA++;   // forward
//   else pulse_countA--;         //backward
// }

// void readEncoderB() {
//   int a = digitalRead(ENCB);
//   int b = digitalRead(PMB);
//   if (a == b) pulse_countB++;   // forward
//   else pulse_countB--;         //backward
// }

// /*void Encoder_Get(){
//   if (millis() - lastTime >= 1000) {  
//     noInterrupts();
//     long countA = pulse_countA;
//     long countB = pulse_countB;
//     pulse_countA = 0;
//     pulse_countB = 0;
//     interrupts();

//     rpmA = ((float)countA / (pulses_per_rev * gear_ratio)) * 280;
//     rpmB = ((float)countB / (pulses_per_rev * gear_ratio)) * 280;
//     Serial.print("SpeedA: ");
//     Serial.print(rpmA);
//     Serial.println(" RPM");
//     lastTime = millis();
//   }
// }*/

// bool encoderSampleRPM(float& rpmA, float& rpmB) {
//   if (millis() - lastTimeMs < 10) return false;

//   noInterrupts();
//   long countA = pulse_countA;
//   long countB = pulse_countB;
//   pulse_countA = 0;
//   pulse_countB = 0;
//   interrupts();

//   const float ticks_per_rev = pulses_per_rev * gear_ratio; // 输出轴一圈脉冲数
//   float dt = (millis() - lastTimeMs) / 1000.0;
//   rpmA = (countA / ticks_per_rev) / dt * 60.0;
//   rpmB = -(countB / ticks_per_rev) / dt * 60.0;


//   lastTimeMs = millis();
//   return true;
// }