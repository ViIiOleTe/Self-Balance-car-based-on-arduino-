#pragma once
#include "Config.h"

static inline int balance_pd(float angle_meas, float gyro_meas);
static inline int velocity_pi_update(long speed_pulses_40ms, long target_pulses_40ms);
static inline int turn_pd(float gyro_z);
void PID_PWMcount(int *pwmL, int *pwmR);
// void get_datax(float &angX, float &accX, float &gyroX);
void Bluetooth_Debug();
void sendTelemetry();
void updateAvoidanceTargetAngle();
