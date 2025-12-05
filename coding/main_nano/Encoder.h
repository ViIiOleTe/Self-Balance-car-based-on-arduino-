#pragma once
#include "Config.h"

void Encoder_init();

bool encoderSampleRPM(float& rpmA, float& rpmB);

void readEncoderA();
void readEncoderB();