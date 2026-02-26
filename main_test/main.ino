#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "Imu.h"
#include "Bt.h"
#include "Ultrasonic.h"
#include "PID.h"
#include "test.h"

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
static const uint32_t TEST_PERIOD_MS = 100;

void setup() {
  Serial.begin(115200);

  Encoder_init();
  Motor_Init();
  init_mpu();
  // ultra_init();
  btBegin();
  Test_Init();
  lastControlUs = micros();
}

void loop() {
  
//5ms loop
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    // 算控制 + 输出电机（这两句尽量保持轻量）
    PID_PWMcount(&PWM_A,&PWM_B);
    Wheel_drive(PWM_A,PWM_B, SCA, SCB);
  }

  if ((uint32_t)(nowMs - lastControlUs) >= TEST_PERIOD_MS) {
        // 防止累积漂移：按固定周期推进
    lastControlUs += TEST_PERIOD_MS;
    // === 读取 IMU ===
    float AngleX, AccX, GyX;
    get_dataX(AngleX, AccX, GyX);

    // === 测试模块 ===
    Test_Update(AngleX);
  }
  encoderSamplePulses(AS,BS,40);
  // ===== 2) 非实时任务：尽量别太耗时、别 delay =====
  // Bluetooth_Debug();

  // if (millis() - lastSend > 500) {
  //   lastSend = millis();
  //   sendTelemetry();
  // }
  Test_HandleSerial();
}
