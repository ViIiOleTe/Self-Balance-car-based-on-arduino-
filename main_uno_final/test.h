#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

// 初始化测试模块（会配置串口、初始化自动模式状态）
void Test_Init();

// 在 loop 里调用：用于“上电延时后自动开始”
void Test_AutoTask();

// 在你的调度里周期调用：喂给测试系统一个角度（deg）
void Test_Update(float angleX_deg);

// 插线后使用：串口命令（DUMP / REPORT / CLEAR）
void Test_HandleSerial();

#endif