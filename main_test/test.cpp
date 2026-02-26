#include "test.h"

// ================= 配置 =================
#define TEST_BAUD 9600

// ================= 状态 =================
static bool testRunning = false;

static uint32_t startTime = 0;


static double sumSq = 0.0;
static uint32_t sampleCount = 0;


static float thetaRef = -0.3f;   // 先写死，后面可改成 extern

void Test_Init() {
  Serial.println(F("# Test system ready"));
  Serial.println(F("# cmd: START / STOP / REPORT"));
}

static void printHeader() {
  Serial.println(F("t_ms,AngleX_deg,thetaRef_deg,err_deg"));
}

static void startTest() {
  testRunning = true;

  startTime = millis();
  sumSq = 0.0;
  sampleCount = 0;

  printHeader();

  Serial.println(F("# TEST START"));
}

static void stopTest() {
  testRunning = false;
  Serial.println(F("# TEST STOP"));
}

static void reportTest() {
  if (sampleCount == 0) {
    Serial.println(F("# No data"));
    return;
  }

  double rms = sqrt(sumSq / sampleCount);

  Serial.print(F("# RMS(deg) = "));
  Serial.println(rms, 6);
}

void Test_Update(float angleX) {

  if (!testRunning) return;

  uint32_t t = millis() - startTime;

  float err = thetaRef - angleX;

  sumSq += (double)err * (double)err;
  sampleCount++;

  Serial.print(t);
  Serial.print(',');
  Serial.print(angleX, 4);
  Serial.print(',');
  Serial.print(thetaRef, 4);
  Serial.print(',');
  Serial.println(err, 4);
}

void Test_HandleSerial() {

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "START") {
    startTest();
  }
  else if (cmd == "STOP") {
    stopTest();
  }
  else if (cmd == "REPORT") {
    reportTest();
  }
  else {
    Serial.println(F("# Unknown cmd"));
  }
}