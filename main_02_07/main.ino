#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Imu.h"
#include "Bt.h"
#include "Ultrasonic.h"
#include "PID.h"

// ===== definitions for extern =====
int PWM;
int PWM_A = 0;
int PWM_B = 0;
int SCA = Start_PWMA;
int SCB = Start_PWMB;
unsigned long lastSend = 0;
long AS, BS;
static uint32_t lastControlUs = 0;
static const uint32_t CONTROL_PERIOD_US = 5000; // 5ms = 5000us

void setup() {
  Serial.begin(9600);

  Encoder_init();
  Motor_Init();
  init_mpu();
  // ultra_init();
  btBegin();
  ultra_init();

  lastControlUs = micros();
}

void loop() {

  static uint32_t lastUltraMs = 0;

  if (millis() - lastUltraMs >= 40) {   // 40Hz
    lastUltraMs = millis();
// Serial.println(GetDistance());
serial_Debug();
Wheel_drive(PWM_A,PWM_B, SCA, SCB);
  //   updateAvoidanceTargetAngle();     // 更新 Balance_Angle / crash
  }
// // //5ms loop
//   uint32_t nowUs = micros();
//   if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
//     // 防止累积漂移：按固定周期推进
//     lastControlUs += CONTROL_PERIOD_US;

//     // 算控制 + 输出电机（这两句尽量保持轻量）
//     PID_PWMcount(&PWM_A,&PWM_B);
//     Wheel_drive(PWM_A,PWM_B, SCA, SCB);
//   }
//   encoderSamplePulses(AS,BS,40);
//   // ===== 2) 非实时任务：尽量别太耗时、别 delay =====
//   Bluetooth_Debug();

//   if (millis() - lastSend > 500) {
//     lastSend = millis();
//     sendTelemetry();
//   }

}
