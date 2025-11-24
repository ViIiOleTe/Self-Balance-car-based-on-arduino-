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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Encoder_init();
  Motor_Init();
  init_mpu();
  ultra_init();
  btBegin();
}

void loop() {
  //keyboard control speed
  serial_Debug();
  Wheel_drive(PWM, SCA, SCB);

  //get and print speed value
  float rpmA, rpmB;
  if (encoderSampleRPM(rpmA, rpmB)) {
    Serial.print("SpeedA: "); Serial.print(rpmA); Serial.print(" RPM, ");
    Serial.print("SpeedB: "); Serial.print(rpmB); Serial.println(" RPM");
  }
  
  //print mpu6050 value
  imuPollAndPrint();
  float AngleX, AccX, GyX; 
  get_dataX(AngleX, AccX, GyX);

  // //print ultrasonic value
  // float dist = GetDistance();
  // if (dist > 0) {
  //   Serial.print("Distance: ");
  //   Serial.print(dist);
  //   Serial.println(" cm");
  // } else {
  //   Serial.println("No echo");
  // }
  //PID
  PWM = PID_PWMcount();
  Bluetooth_Debug();
}
