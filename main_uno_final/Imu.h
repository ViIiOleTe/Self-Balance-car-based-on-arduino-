#pragma once
#include "Config.h"
void init_mpu();
void get_kf(int16_t &ax, int16_t &ay, int16_t &az,
            int16_t &gx, int16_t &gy, int16_t &gz);
// void init_mpu();
// void get_angle(float &x, float &y, float &z);
// void get_acc(float &x, float &y, float &z);
// void get_gyro(float &x, float &y, float &z);
// void get_dataX(float &Ang_x, float &Acc_x, float &Gyro_x);
// void get_dataZ(float &Ang_z, float &Acc_z, float &Gyro_z);
// bool imuPollAndPrint();