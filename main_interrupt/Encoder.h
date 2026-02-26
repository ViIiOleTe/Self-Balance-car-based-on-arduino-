#pragma once
#include "Config.h"

void Encoder_init();
bool encoderSampleSum(long& sum, uint16_t window_ms);
bool encoderSamplePulses(long& pA, long& pB, uint16_t window_ms);
// bool encoderSampleRPM(float& rpmA, float& rpmB);

void readEncoderA();
void readEncoderB();