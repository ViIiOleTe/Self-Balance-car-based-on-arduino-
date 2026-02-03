#pragma once
#include <Arduino.h>

// ===== Pins =====
#define AIN1 12
#define AIN2 13
#define BIN1 7
#define BIN2 6
#define PWMA 11
#define PWMB 10
#define ENCA 2         // encoder A channel
#define ENCB 3         // encoder B channel
#define PMA  A0        // phase B for motor A
#define PMB  A1        // phase B for motor B
// #define SDA_Pin 
// #define SCL_Pin 
#define Trig 9
#define Echo 8
#define BT_RX 4   // HC-05 TX --> Arduino pin 4
#define BT_TX 5   // HC-05 RX --> Arduino pin 5

// ===== Values =====
#define Start_PWMA 9   // start compensation A
#define Start_PWMB 10  // start compensation B

// ===== Encoder constants =====
const int   pulses_per_rev = 11;     // per motor shaft
const float gear_ratio     = 20; //89.6/4.667;  // gearbox ratio

// ===== Shared variables (defined in main.ino) =====
extern int PWM;
extern int SCA;
extern int SCB;

