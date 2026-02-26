#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

// 初始化测试模块
void Test_Init();

// 在 5ms 控制环里调用
void Test_Update(float angleX);

// 串口命令处理（start / stop / report）
void Test_HandleSerial();

#endif