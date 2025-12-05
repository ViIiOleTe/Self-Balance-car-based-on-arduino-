#pragma once
#include "Config.h"

void init_mpu();
void get_angle(float &x, float &y, float &z);
void get_acc(float &x, float &y, float &z);
void get_gyro(float &x, float &y, float &z);
void get_dataX(float &Ang_x, float &Acc_x, float &Gyro_x);
bool imuPollAndPrint();