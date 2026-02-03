#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Imu.h"
#include "Bt.h"
#include "Ultrasonic.h"
#include "PID.h"

// ===== definitions for extern =====
int PWM = 0;
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

  lastControlUs = micros();
}

void loop() {
  // ===== 1) 5ms 控制调度（不用中断）=====
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    // 防止累积漂移：按固定周期推进
    lastControlUs += CONTROL_PERIOD_US;

    // 算控制 + 输出电机（这两句尽量保持轻量）
    PWM = PID_PWMcount();
    Wheel_drive(PWM, SCA, SCB);
  }
  encoderSamplePulses(AS,BS,40);
  // ===== 2) 非实时任务：尽量别太耗时、别 delay =====
  Bluetooth_Debug();

  if (millis() - lastSend > 500) {
    lastSend = millis();
    sendTelemetry();
  }

}

// #include "Config.h"
// #include "Motor.h"
// #include "Encoder.h"
// #include "Imu.h"
// #include "Bt.h"
// #include "Ultrasonic.h"
// #include "PID.h"

// // ===== definitions for extern =====
// int PWM = 0;
// int SCA = Start_PWMA;
// int SCB = Start_PWMB;
// unsigned long lastSend = 0;

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   Encoder_init();
//   Motor_Init();
//   init_mpu();
//   // ultra_init();
//   btBegin();
// }

// void loop() {
//   //keyboard control speed

//   // serial_Debug();
//   Wheel_drive(PWM, SCA, SCB);

//   //get and print speed value
//   // float rpmA, rpmB;
//   // if (encoderSampleRPM(rpmA, rpmB)) {
//   //   Serial.print("SpeedA: "); Serial.print(rpmA); Serial.print(" RPM, ");
//   //   Serial.print("SpeedB: "); Serial.print(rpmB); Serial.println(" RPM");
//   // }
  
//   //print mpu6050 value
//   // imuPollAndPrint();
//   // float AngleX, AccX, GyX; 
//   // get_dataX(AngleX, AccX, GyX);

//   // //print ultrasonic value
//   // float dist = GetDistance();
//   // if (dist > 0) {
//   //   Serial.print("Distance: ");
//   //   Serial.print(dist);
//   //   Serial.println(" cm");
//   // } else {
//   //   Serial.println("No echo");
//   // }
//   //PID
//   // PWM = PID_PWMcount();
//   Bluetooth_Debug();
  
//   if (millis() - lastSend > 500) {
//         lastSend = millis();
//         sendTelemetry();
//     }
// }
