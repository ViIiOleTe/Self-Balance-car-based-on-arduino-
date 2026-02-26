#include "Ultrasonic.h"
#include "PID.h"

// 你原来应该在 Ultrasonic.h 里定义了 Trig/Echo 引脚宏或变量
// 例如：#define Trig 7  #define Echo 8

void ultra_init() {
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    digitalWrite(Trig, LOW);
}

// ===== 非阻塞超声：状态机 =====
enum UltraState {
    U_IDLE,
    U_TRIG_LOW,
    U_TRIG_HIGH,
    U_WAIT_RISE,
    U_WAIT_FALL
};

static volatile float g_distance_cm = -1.0f;   // 最近一次距离（cm），-1 表示无效
static UltraState g_state = U_IDLE;
static uint32_t g_tMarkUs = 0;
static uint32_t g_tRiseUs = 0;

// 可调：最大测距范围（越大越慢）
// 12000us ≈ 2m；6000us≈1m；18000us≈3m（不推荐太大）
static const uint32_t ULTRA_TIMEOUT_US = 12000;

// 可调：两次测距之间的间隔（避免太频繁）
// 建议 60~200ms；平衡车避障一般 100~200ms 足够
static const uint16_t ULTRA_PERIOD_MS = 150;

static uint32_t g_lastStartMs = 0;

// 在 loop() 里每次调用一次：永不阻塞
void ultra_update() {
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    switch (g_state) {

        case U_IDLE:
            // 控制测距频率：到点才开始下一次
            if ((uint32_t)(nowMs - g_lastStartMs) >= ULTRA_PERIOD_MS) {
                g_lastStartMs = nowMs;

                digitalWrite(Trig, LOW);
                g_tMarkUs = nowUs;
                g_state = U_TRIG_LOW;
            }
            break;

        case U_TRIG_LOW:
            // 低电平保持 >=2us
            if ((uint32_t)(nowUs - g_tMarkUs) >= 2) {
                digitalWrite(Trig, HIGH);
                g_tMarkUs = nowUs;
                g_state = U_TRIG_HIGH;
            }
            break;

        case U_TRIG_HIGH:
            // 高电平保持 >=10us
            if ((uint32_t)(nowUs - g_tMarkUs) >= 10) {
                digitalWrite(Trig, LOW);
                g_tMarkUs = nowUs;      // 用于等待上升沿超时计时
                g_state = U_WAIT_RISE;
            }
            break;

        case U_WAIT_RISE:
            // 等待 Echo 变高
            if (digitalRead(Echo) == HIGH) {
                g_tRiseUs = micros();
                g_state = U_WAIT_FALL;
            } else if ((uint32_t)(nowUs - g_tMarkUs) > ULTRA_TIMEOUT_US) {
                // 超时：无回波
                g_distance_cm = -1.0f;
                g_state = U_IDLE;
            }
            break;

        case U_WAIT_FALL:
            // 等待 Echo 变低
            if (digitalRead(Echo) == LOW) {
                uint32_t tFallUs = micros();
                uint32_t duration = tFallUs - g_tRiseUs;

                // 计算距离 cm
                g_distance_cm = (float)duration * 0.034f * 0.5f;

                g_state = U_IDLE;
            } else if ((uint32_t)(micros() - g_tRiseUs) > ULTRA_TIMEOUT_US) {
                // 超时：回波太长/异常
                g_distance_cm = -1.0f;
                g_state = U_IDLE;
            }
            break;
    }
}

// ===== 兼容你的旧接口：不测距，只返回最近缓存值（不阻塞）=====
float GetDistance() {
    return g_distance_cm;
}

// #include "Ultrasonic.h"
// #include "PID.h"

// void ultra_init() {
//     pinMode(Trig, OUTPUT);
//     pinMode(Echo, INPUT);
// }

// float GetDistance() {
//     // trigger 10µs
//     digitalWrite(Trig, LOW);
//     delayMicroseconds(2);
//     digitalWrite(Trig, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(Trig, LOW);

//     // read
//     long duration = pulseIn(Echo, HIGH, 30000);   // 30ms overtime

//     if (duration == 0) {
//         return -1.0; // overtime
//     }

//     float distance_cm = duration * 0.034f / 2.0f;
//     return distance_cm;
// }
