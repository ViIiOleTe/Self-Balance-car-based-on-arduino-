#include "Encoder.h"
#include <Arduino.h>

// volatile 供中断修改
static volatile long pulse_countA = 0;
static volatile long pulse_countB = 0;

static unsigned long lastTimeMs = 0;

void Encoder_init(){
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PMA, INPUT);
  pinMode(PMB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, RISING);
}

void readEncoderA() {
  int a = digitalRead(ENCA);
  int b = digitalRead(PMA);
  if (a == b) pulse_countA++;   // forward
  else pulse_countA--;         //backward
}

void readEncoderB() {
  int a = digitalRead(ENCB);
  int b = digitalRead(PMB);
  if (a == b) pulse_countB++;   // forward
  else pulse_countB--;         //backward
}

/*void Encoder_Get(){
  if (millis() - lastTime >= 1000) {  
    noInterrupts();
    long countA = pulse_countA;
    long countB = pulse_countB;
    pulse_countA = 0;
    pulse_countB = 0;
    interrupts();

    rpmA = ((float)countA / (pulses_per_rev * gear_ratio)) * 280;
    rpmB = ((float)countB / (pulses_per_rev * gear_ratio)) * 280;
    Serial.print("SpeedA: ");
    Serial.print(rpmA);
    Serial.println(" RPM");
    lastTime = millis();
  }
}*/

bool encoderSampleRPM(float& rpmA, float& rpmB) {
  if (millis() - lastTimeMs < 10) return false;

  noInterrupts();
  long countA = pulse_countA;
  long countB = pulse_countB;
  pulse_countA = 0;
  pulse_countB = 0;
  interrupts();

  const float ticks_per_rev = pulses_per_rev * gear_ratio; // 输出轴一圈脉冲数
  float dt = (millis() - lastTimeMs) / 1000.0;
  rpmA = -(countA / ticks_per_rev) / dt * 60.0;
  rpmB = (countB / ticks_per_rev) / dt * 60.0;


  lastTimeMs = millis();
  return true;
}