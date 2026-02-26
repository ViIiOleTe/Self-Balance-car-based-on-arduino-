#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Imu.h"
#include "Bt.h"
#include "Ultrasonic.h"
#include "PID.h"
#include "timer2.h"


// ===== definitions for extern =====
int PWM;
int PWM_A = 0;
int PWM_B = 0;
int SCA = Start_PWMA;
int SCB = Start_PWMB;
unsigned long lastSend = 0;
long AS, BS;

void setup() {
  Serial.begin(9600);
  timer2_init_5ms();
  Encoder_init();
  Motor_Init();
  init_mpu();
  btBegin();
  ultra_init();
}

void loop() {
  // ===== 1) 5ms control =====
  while (nowTick != lastTick) {
    lastTick++;
    noInterrupts();
    nowTick = g_tick5ms;
    interrupts();

    PID_PWMcount(&PWM_A, &PWM_B);
    Wheel_drive(PWM_A, PWM_B, SCA, SCB);
  }
  
  // ===== ultrasonic =====
  static uint32_t lastUltraMs = 0;
  ultra_update();
  if (millis() - lastUltraMs >= 200){//timer 1 soft timming
    lastUltraMs = millis();
    updateAvoidanceTargetAngle();
  }

  // ===== encoder =====
  encoderSamplePulses(AS, BS, 40);

  // ===== BT =====
  Bluetooth_Debug();

  // ===== remote =====
  if (millis() - lastSend > 500) {

    lastSend = millis();
    sendTelemetry();
  }
}

