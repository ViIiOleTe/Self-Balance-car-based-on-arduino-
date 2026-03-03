#pragma once
#include "Config.h"

void Motor_Init();
void serial_Debug();
void Wheel_drive(int pwm_a, int pwm_b, int SCA, int SCB);
