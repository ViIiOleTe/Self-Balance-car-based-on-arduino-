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

 // ===== 1) 编码器测试 =====
  static uint32_t lastEncTick = 0;

  uint32_t nowTick;
  noInterrupts();
  nowTick = g_tick5ms;
  interrupts();
  
  if (nowTick - lastEncTick >= 40) { // 200ms

    lastEncTick = nowTick;

    long a, b;
    serial_Debug();
    Wheel_drive(PWM_A, PWM_B, SCA, SCB);
    if (encoderSamplePulses(a, b, 20)) {

      Serial.print("ENC A: ");
      Serial.print(a);
      Serial.print("  B: ");
      Serial.println(b);
    }
  }

  // // ===== 1) 5ms 控制任务 =====
  // if (g_control5ms) {

  //   noInterrupts();
  //   g_control5ms = false;
  //   interrupts();

  //   PID_PWMcount(&PWM_A, &PWM_B);
  //   Wheel_drive(PWM_A, PWM_B, SCA, SCB);
  // }


  // // ===== 2) 40ms 超声 =====
  // static uint32_t lastUltraMs = 0;

  // if (millis() - lastUltraMs >= 40) {

  //   lastUltraMs = millis();
  //   Serial.print();
  //   serial_Debug();
  //   updateAvoidanceTargetAngle();
  // }


  // // ===== 3) 编码器采样 =====
  // encoderSamplePulses(AS, BS, 40);


  // // ===== 4) 蓝牙 =====
  // Bluetooth_Debug();


  // // ===== 5) 遥测 =====
  // if (millis() - lastSend > 500) {

  //   lastSend = millis();
  //   sendTelemetry();
  // }
}

