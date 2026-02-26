#include "Imu.h"
#include <Wire.h>
#include <MPU6050_tockn.h>

static MPU6050 mpu(Wire);
static uint32_t lastPrintMs = 0;

void init_mpu(){
  Wire.begin();
  mpu.begin(); 
  mpu.calcGyroOffsets(-1.21, 0.74, 0.21);//need tuning
}

void get_angle(float &x, float &y, float &z){
  mpu.update();
  x=mpu.getAngleX();
  y=mpu.getAngleY();
  z=mpu.getAngleZ();
}

void get_acc(float &x, float &y, float &z){
  mpu.update();
  x=mpu.getAccX();
  y=mpu.getAccY();
  z=mpu.getAccZ();
}

void get_gyro(float &x, float &y, float &z){
  mpu.update();
  x=mpu.getGyroX();
  y=mpu.getGyroY();
  z=mpu.getGyroZ();
}

void get_dataX(float &Ang_x, float &Acc_x, float &Gyro_x){
  mpu.update();
  Ang_x=mpu.getAngleX();
  Acc_x=mpu.getAccX();
  Gyro_x=mpu.getGyroX();
}

bool imuPollAndPrint() {
  mpu.update();
  uint32_t now = millis();
  if (now - lastPrintMs < 1000) return false;

  lastPrintMs = now;
  Serial.print("angleX: "); Serial.print(mpu.getAngleX());
  Serial.print("  AccX: ");  Serial.print(mpu.getAccX());
  Serial.print("  GyroX: "); Serial.println(mpu.getGyroX());
  return true;
}