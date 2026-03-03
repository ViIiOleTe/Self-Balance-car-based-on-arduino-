#include "Ultrasonic.h"
#include "PID.h"

void ultra_init() {
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
}

float GetDistance() {
    // 触发前：确保 Echo 是 LOW（最多等 2ms，避免卡太久）
    uint32_t t0 = micros();
    while (digitalRead(Echo) == HIGH) {
        if ((uint32_t)(micros() - t0) > 2000) break; // 防死等
    }

    // trigger 10us
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);

    // timeout 用你实际需要的上限，别 30000 一刀切
    unsigned long duration = pulseIn(Echo, HIGH, 5000UL); // 12ms ≈ 2m
    if (duration == 0) return -1.0f;

    return (float)duration * 0.034f * 0.5f;
}

// #include "Ultrasonic.h"
// #include <Arduino.h>

// // ===== 非阻塞超声：轮询增强状态机（不使用中断）=====
// // 关键改动：WAIT_RISE/WAIT_FALL 增加短暂自旋采样窗口，降低近距离漏检

// enum UltraState {
//     U_IDLE,
//     U_TRIG_LOW,
//     U_TRIG_HIGH,
//     U_WAIT_RISE,
//     U_WAIT_FALL
// };

// static volatile float g_distance_cm = -1.0f;
// static UltraState g_state = U_IDLE;

// static uint32_t g_tMarkUs = 0;   // 状态起始计时（us）
// static uint32_t g_tRiseUs = 0;   // Echo 上升沿时间（us）

// // ===== 可调参数 =====
// static const uint32_t ULTRA_TIMEOUT_US = 12000;  // 最大等待（≈2m）
// static const uint16_t ULTRA_PERIOD_MS  = 200;    // 测距周期

// // 自旋采样窗口（关键！）：在等待边沿时，最多忙等这么多 us 来抓短脉冲
// // 建议 100~300us。越大越不容易漏，但越可能影响控制环。
// // 你有 5ms 控制环，先用 200us。
// static const uint32_t SPIN_US = 1000;

// static uint32_t g_lastStartMs = 0;

// // 在最多 spin_us 的时间内反复读 pin，若看到 level 则返回 true，否则 false
// static inline bool spin_wait_level(uint8_t pin, uint8_t level, uint32_t spin_us)
// {
//     uint32_t t0 = micros();
//     while ((uint32_t)(micros() - t0) < spin_us) {
//         if (digitalRead(pin) == level) return true;
//     }
//     return false;
// }

// void ultra_init()
// {
//     pinMode(Trig, OUTPUT);
//     pinMode(Echo, INPUT);
//     digitalWrite(Trig, LOW);
// }

// // loop() 里尽可能频繁调用；内部自己节流为 ULTRA_PERIOD_MS
// void ultra_update()
// {
//     uint32_t nowMs = millis();

//     switch (g_state)
//     {
//         case U_IDLE:
//             // 到点才开始新一轮
//             if ((uint32_t)(nowMs - g_lastStartMs) >= ULTRA_PERIOD_MS) {
//                 g_lastStartMs = nowMs;
//                 digitalWrite(Trig, LOW);
//                 g_tMarkUs = micros();
//                 g_state = U_TRIG_LOW;
//             }
//             break;

//         case U_TRIG_LOW:
//             if ((uint32_t)(micros() - g_tMarkUs) >= 2) {
//                 digitalWrite(Trig, HIGH);
//                 g_tMarkUs = micros();
//                 g_state = U_TRIG_HIGH;
//             }
//             break;

//         case U_TRIG_HIGH:
//             if ((uint32_t)(micros() - g_tMarkUs) >= 10) {
//                 digitalWrite(Trig, LOW);
//                 g_tMarkUs = micros();      // 从这里开始计 WAIT_RISE 超时
//                 g_state = U_WAIT_RISE;
//             }
//             break;

//         case U_WAIT_RISE: {
//             // 先快速看一眼 + 短自旋窗口，降低漏掉上升沿概率
//             if (digitalRead(Echo) == HIGH || spin_wait_level(Echo, HIGH, SPIN_US)) {
//                 g_tRiseUs = micros();
//                 g_state = U_WAIT_FALL;
//             } else if ((uint32_t)(micros() - g_tMarkUs) > ULTRA_TIMEOUT_US) {
//                 g_distance_cm = -1.0f;
//                 g_state = U_IDLE;
//             }
//         } break;

//         case U_WAIT_FALL: {
//             // 同理：短自旋窗口，降低漏掉下降沿概率
//             if (digitalRead(Echo) == LOW || spin_wait_level(Echo, LOW, SPIN_US)) {
//                 uint32_t tFallUs = micros();
//                 uint32_t duration = (uint32_t)(tFallUs - g_tRiseUs);

//                 // 可选：过滤异常过短脉冲（毛刺），比如 < 150us ≈ 2.6cm
//                 // 你如果经常出现“50cm变几厘米”，很可能是毛刺触发了 rise+fall
//                 if (duration < 150) {
//                     g_distance_cm = -1.0f;
//                 } else {
//                     g_distance_cm = (float)duration * 0.034f * 0.5f;
//                 }

//                 g_state = U_IDLE;
//             } else if ((uint32_t)(micros() - g_tRiseUs) > ULTRA_TIMEOUT_US) {
//                 g_distance_cm = -1.0f;
//                 g_state = U_IDLE;
//             }
//         } break;
//     }
// }

// float GetDistance()
// {
//     return g_distance_cm;
// }
