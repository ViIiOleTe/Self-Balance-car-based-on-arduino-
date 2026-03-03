#include "Imu.h"
#include <Wire.h>
#include "MPU6050.h"   // I2Cdevlib 的 MPU6050.h

static MPU6050 mpu;          // 默认地址 0x68
static uint32_t lastPrintMs = 0;

void init_mpu() {
  Wire.begin();
  mpu.initialize();

  // 可选：检查连接
  // if (!mpu.testConnection()) { Serial.println("MPU6050 fail"); }

  // 可选：如果你需要配置量程，在这里设置（不同版本函数名可能略有差异）
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
  // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250 dps
}

void get_kf(int16_t &ax, int16_t &ay, int16_t &az,
            int16_t &gx, int16_t &gy, int16_t &gz) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}
// #include "Imu.h"
// #include <Wire.h>
// #include <MPU6050_tockn.h>

// static MPU6050 mpu(Wire);
// static uint32_t lastPrintMs = 0;

// void init_mpu(){
//   Wire.begin();
//   mpu.begin(); 
//   mpu.calcGyroOffsets(-1.21, 0.74, 0.21);//need tuning
// }

// void get_angle(float &x, float &y, float &z){
//   mpu.update();
//   x=mpu.getAngleX();
//   y=mpu.getAngleY();
//   z=mpu.getAngleZ();
// }

// void get_acc(float &x, float &y, float &z){
//   mpu.update();
//   x=mpu.getAccX();
//   y=mpu.getAccY();
//   z=mpu.getAccZ();
// }

// void get_gyro(float &x, float &y, float &z){
//   mpu.update();
//   x=mpu.getGyroX();
//   y=mpu.getGyroY();
//   z=mpu.getGyroZ();
// }

// void get_dataX(float &Ang_x, float &Acc_x, float &Gyro_x){
//   mpu.update();
//   Ang_x=mpu.getAngleX();
//   Acc_x=mpu.getAccX();
//   Gyro_x=mpu.getGyroX();
// }

// void get_dataZ(float &Ang_z, float &Acc_z, float &Gyro_z){
//   mpu.update();
//   Ang_z=mpu.getAngleZ();
//   Acc_z=mpu.getAccZ();
//   Gyro_z=mpu.getGyroZ();
// }


// bool imuPollAndPrint() {
//   mpu.update();
//   uint32_t now = millis();
//   if (now - lastPrintMs < 1000) return false;

//   lastPrintMs = now;
//   Serial.print("angleX: "); Serial.print(mpu.getAngleX());
//   // Serial.print("  AccX: ");  Serial.print(mpu.getAccX());
//   // Serial.print("  GyroX: "); Serial.println(mpu.getGyroX());
//   return true;
// }