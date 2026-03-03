#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Imu.h"
#include "Bt.h"
#include "Ultrasonic.h"
#include "PID.h"
#include "timer2.h"
#include "test.h"
#include <KalmanFilter.h>

//***************下面是卡尔曼滤波相关变量***************//
static float K1 = 0.05; // 对加速度计取值的权重
static float Q_angle = 0.001, Q_gyro = 0.005;
static float R_angle = 0.5 , C_0 = 1;
static float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
static int addr = 0;

//kalmanfilter
KalmanFilter KalFilter2;
static int16_t ax, ay, az, gx, gy, gz;
static uint32_t lastTestMs = 0;

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
  delay(500);
  btBegin();
  ultra_init();
  // Test_Init();
  delay(500);

  uint32_t nowMs = millis();
  lastTestMs = nowMs;
}

// 建议：在全局区（loop 外）放
static uint32_t lastTick = 0;

void loop() {

 // ==============================
 // 0) loop total timing (max)
 // ==============================
 static uint32_t max_loop_us = 0;
 uint32_t loop_t0 = micros();

  // ==============================
  // 1) 5ms control (max)
  // ==============================
  static uint32_t max_ctrl_us = 0;
  // 读当前 tick（原子）
  uint32_t ticks;
  noInterrupts();
  ticks = g_tick5ms;
  interrupts();

  // 可选：防止极端卡顿积压太多 tick（最多补 8 次 = 40ms）
  uint8_t catchup = 0;

  while (lastTick != ticks && catchup < 8) {
    lastTick++;
    catchup++;

    // uint32_t t0 = micros();
    PID_PWMcount(&PWM_A, &PWM_B);
    Wheel_drive(PWM_A, PWM_B, SCA, SCB);
    // uint32_t dt = micros() - t0;

    // if (dt > max_ctrl_us) max_ctrl_us = dt;
  }

  // ==============================
  // 2) ultrasonic update (max)
  // ==============================

  static uint32_t max_ultra_200ms_us  = 0;

  static uint32_t lastUltraMs = 0;
  if (millis() - lastUltraMs >= 200) {
    lastUltraMs = millis();

    // uint32_t t0 = micros();
    updateAvoidanceTargetAngle();
  }
  //   uint32_t dt = micros() - t0;
  //   if (dt > max_ultra_200ms_us) max_ultra_200ms_us = dt;
  // }

 // ==============================
 // 3) BT debug (max)
 // ==============================
//  static uint32_t max_bt_us = 0;
//  {
//    uint32_t t0 = micros();
   Bluetooth_Debug();
//    uint32_t dt = micros() - t0;
//    if (dt > max_bt_us) max_bt_us = dt;
//  }

 // ==============================
 // 4) telemetry (max)
 // ==============================
//  static uint32_t max_tel_us = 0;

//  if (millis() - lastSend > 500) {
//    lastSend = millis();
  
//    uint32_t t0 = micros();
//   //  sendTelemetry();
//    uint32_t dt = micros() - t0;
//    if (dt > max_tel_us) max_tel_us = dt;
//  }

//  // ==============================
//  // 5) finish loop total timing
//  // ==============================
//  uint32_t loop_dt = micros() - loop_t0;
//  if (loop_dt > max_loop_us) max_loop_us = loop_dt;

//  // ==============================
//  // 6) print once per second
//  // ==============================
//  static uint32_t lastPrintMs = 0;
//  if (millis() - lastPrintMs >= 1000) {
//    lastPrintMs += 1000;

//    Serial.print("MAX us | LOOP=");
//    Serial.print(max_loop_us);
//    Serial.print(" CTRL=");
//    Serial.print(max_ctrl_us);
//    Serial.print(" U200=");
//    Serial.print(max_ultra_200ms_us);
//    Serial.print(" BT=");
//    Serial.print(max_bt_us);
//    Serial.print(" TEL=");
//    Serial.print(max_tel_us);
//     Serial.print(" DIST=");
//    Serial.println(GetDistance());
//    // reset for next window
//    max_loop_us = 0;
//    max_ctrl_us = 0;
//    max_ultra_200ms_us  = 0;
//    max_bt_us   = 0;
//    max_tel_us  = 0;
  // }
}
