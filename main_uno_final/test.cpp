#include "test.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include "pid.h"

// ================= 配置 =================
#define TEST_BAUD 9600

// ===== 自动测试参数 =====
static const bool     AUTO_ENABLE       = true;     // 开关
static const uint32_t AUTO_DURATION_MS  = 10000;     // 自动测试时长：例如 6 秒
static const uint32_t AUTO_ARM_DELAY_MS = 5000;     // 上电后延时开始（让 IMU/滤波稳定），例如 1.5 秒

// EEPROM (UNO = 1024 bytes)
static const uint16_t LOG_MAGIC = 0xA55A;
static const int EEPROM_SIZE = 1024;

// Header layout
// 0..1: magic
// 2..3: count
// 4..5: dt_ms
static const int HDR_MAGIC_ADDR = 0;
static const int HDR_COUNT_ADDR = 2;
static const int HDR_DT_ADDR    = 4;
static const int DATA_START     = 6;

// 记录周期：建议 20/40/50ms（EEPROM 寿命 & 数据长度折中）
static const uint16_t LOG_DT_MS = 100;

// 角度定点缩放：deg * 1000 -> mdeg (int16)
static const float ANG_SCALE = 1000.0f;

// ================= 状态 =================
static bool testRunning = false;
static uint32_t startTime = 0;     // 实际开始记录的时刻
static uint32_t lastLogMs = 0;

static double sumSq = 0.0;
static uint32_t sampleCount = 0;

// 你的参考角（仍然保留）
static float thetaRef = -2.3f;

// 自动模式状态
static bool autoArmed = false;
static bool autoDone  = false;
static uint32_t bootMs = 0;

// ================= EEPROM 记录结构 =================
struct LogRec {
  uint16_t t_unit;     // time index in units of LOG_DT_MS
  int16_t  angle_mdeg; // angle * 1000
  int16_t  ref_mdeg;   // ref * 1000
  int16_t  err_mdeg;   // (ref-angle)*1000
};

static int maxRecords() {
  return (EEPROM_SIZE - DATA_START) / (int)sizeof(LogRec);
}

static void eepromWriteHeader(uint16_t magic, uint16_t count, uint16_t dt_ms) {
  EEPROM.put(HDR_MAGIC_ADDR, magic);
  EEPROM.put(HDR_COUNT_ADDR, count);
  EEPROM.put(HDR_DT_ADDR, dt_ms);
}

static void eepromReadHeader(uint16_t &magic, uint16_t &count, uint16_t &dt_ms) {
  EEPROM.get(HDR_MAGIC_ADDR, magic);
  EEPROM.get(HDR_COUNT_ADDR, count);
  EEPROM.get(HDR_DT_ADDR, dt_ms);
}

static void eepromClearMagic() {
  uint16_t zero = 0;
  EEPROM.put(HDR_MAGIC_ADDR, zero);
}

static void printHeaderCSV() {
  Serial.println(F("t_ms,AngleX_deg,thetaRef_deg,err_deg"));
}

// ================= Test 控制 =================
static void startTest() {
  testRunning = true;
  // Balance_Angle = -2.3+15;
  // delay(300);
  // Balance_Angle = -2.3;
  startTime = millis();
  lastLogMs = startTime;

  sumSq = 0.0;
  sampleCount = 0;

  // 先把 magic 清掉，避免“半截数据”被当有效
  eepromClearMagic();
  // 写 dt & count=0（magic=0 表示还没完成）
  eepromWriteHeader(0, 0, LOG_DT_MS);

  Serial.println(F("# TEST START (AUTO, logging to EEPROM)"));
}

static void stopTest() {
  testRunning = false;

  // 标记数据有效
  uint16_t magic, count, dt_ms;
  eepromReadHeader(magic, count, dt_ms);
  (void)magic;
  eepromWriteHeader(LOG_MAGIC, count, dt_ms);

  Serial.println(F("# TEST STOP (AUTO, log sealed)"));
}

static void reportTest() {
  if (sampleCount == 0) {
    Serial.println(F("# No samples in RAM"));
    return;
  }
  double rms = sqrt(sumSq / sampleCount);
  Serial.print(F("# RMS(deg) = "));
  Serial.println(rms, 6);

  uint16_t magic, count, dt_ms;
  eepromReadHeader(magic, count, dt_ms);
  Serial.print(F("# EEPROM count = "));
  Serial.print(count);
  Serial.print(F(", dt_ms = "));
  Serial.println(dt_ms);
}

// 写一条记录到 EEPROM
static void logOne(uint32_t nowMs, float angleX_deg) {
  uint16_t magic, count, dt_ms;
  eepromReadHeader(magic, count, dt_ms);
  (void)magic;

  int m = maxRecords();
  if ((int)count >= m) {
    testRunning = false;
    Serial.println(F("# EEPROM FULL -> auto stop"));
    eepromWriteHeader(LOG_MAGIC, count, dt_ms);
    autoDone = true;
    return;
  }

  float err = thetaRef - angleX_deg;

  sumSq += (double)err * (double)err;
  sampleCount++;

  LogRec r;
  uint32_t t_ms = nowMs - startTime;
  r.t_unit = (uint16_t)(t_ms / LOG_DT_MS);

  r.angle_mdeg = (int16_t)lroundf(angleX_deg * ANG_SCALE);
  r.ref_mdeg   = (int16_t)lroundf(thetaRef  * ANG_SCALE);
  r.err_mdeg   = (int16_t)lroundf(err       * ANG_SCALE);

  int addr = DATA_START + (int)count * (int)sizeof(LogRec);
  EEPROM.put(addr, r);

  count++;
  EEPROM.put(HDR_COUNT_ADDR, count);
}

// ================= 你在控制循环里调用它 =================
void Test_Update(float angleX_deg) {
  // 自动模式：到时间自动 stop
  if (AUTO_ENABLE && testRunning && !autoDone) {
    uint32_t now = millis();
    if ((uint32_t)(now - startTime) >= AUTO_DURATION_MS) {
      stopTest();
      autoDone = true;
      return;
    }
  }

  if (!testRunning) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastLogMs) < LOG_DT_MS) return;
  lastLogMs += LOG_DT_MS; // 固定步进，避免漂移

  logOne(now, angleX_deg);
}

// 在 loop 里也可以调用一次（可选）
// 用于处理“上电延时后自动开始”
void Test_AutoTask() {
  if (!AUTO_ENABLE) return;
  if (autoDone) return;
  // if (Serial) return;

  uint32_t now = millis();

  // 上电后延时一段时间再开始
  if (!autoArmed) {
    if ((uint32_t)(now - bootMs) >= AUTO_ARM_DELAY_MS) {
      autoArmed = true;
      startTest();
    }
  }
}

// ================= 导出 CSV（插线后用） =================
static void dumpEEPROM() {
  uint16_t magic, count, dt_ms;
  eepromReadHeader(magic, count, dt_ms);

  if (magic != LOG_MAGIC) {
    Serial.println(F("# No valid log (magic mismatch)."));
    return;
  }

  Serial.print(F("# DUMP count="));
  Serial.print(count);
  Serial.print(F(", dt_ms="));
  Serial.println(dt_ms);

  printHeaderCSV();

  double s = 0.0;
  uint32_t n = 0;

  for (uint16_t i = 0; i < count; i++) {
    int addr = DATA_START + (int)i * (int)sizeof(LogRec);
    LogRec r;
    EEPROM.get(addr, r);

    uint32_t t_ms = (uint32_t)r.t_unit * (uint32_t)dt_ms;
    float angle = r.angle_mdeg / ANG_SCALE;
    float ref   = r.ref_mdeg   / ANG_SCALE;
    float err   = r.err_mdeg   / ANG_SCALE;

    Serial.print(t_ms); Serial.print(',');
    Serial.print(angle, 3); Serial.print(',');
    Serial.print(ref,   3); Serial.print(',');
    Serial.println(err, 3);

    s += (double)err * (double)err;
    n++;
  }

  if (n > 0) {
    double rms = sqrt(s / (double)n);
    Serial.print(F("# RMS_from_EEPROM(deg) = "));
    Serial.println(rms, 6);
  }
}

static void clearEEPROMLog() {
  eepromWriteHeader(0, 0, LOG_DT_MS);
  Serial.println(F("# Cleared header (invalidated log)"));
}

// ================= 初始化 & 串口命令（插线后用） =================
void Test_Init() {
  Serial.begin(TEST_BAUD);

  bootMs = millis();
  autoArmed = false;
  autoDone  = false;

  Serial.println(F("# Test EEPROM logger ready"));
  Serial.println(F("# AUTO: will START after arm delay, STOP after duration"));
  Serial.print(F("# arm_delay_ms=")); Serial.println(AUTO_ARM_DELAY_MS);
  Serial.print(F("# duration_ms="));  Serial.println(AUTO_DURATION_MS);
  Serial.println(F("# cmd: DUMP / REPORT / CLEAR (optional)"));

  // 注意：不要在这里立刻 start，等 AUTO_ARM_DELAY_MS 过去再开始
}

void Test_HandleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "DUMP") {
    dumpEEPROM();
  }
  else if (cmd == "REPORT") {
    reportTest();
  }
  else if (cmd == "CLEAR") {
    clearEEPROMLog();
  }
  else {
    Serial.println(F("# Unknown cmd (use DUMP/REPORT/CLEAR)"));
  }
}