#pragma once
#include "Config.h"

void init_mpu();
void get_angle(float &x, float &y, float &z);
void get_acc(float &x, float &y, float &z);
void get_gyro(float &x, float &y, float &z);
void get_dataX(float &Ang_x, float &Acc_x, float &Gyro_x);
void get_dataZ(float &Ang_z, float &Acc_z, float &Gyro_z);
bool imuPollAndPrint();