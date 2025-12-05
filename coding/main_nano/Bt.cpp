#include "Bt.h"
#include <HardwareSerial.h>

HardwareSerial BT(1);   // 使用 UART1（ESP32 有 UART0/1/2）

void btBegin() {
    // 将 UART1 映射到 GPIO 4（RX）和 GPIO 5（TX）
    BT.begin(9600, SERIAL_8N1, BT_RX, BT_TX);
}

bool btAvailable() {
    return BT.available();
}

char btRead() {
    return BT.read();
}

void btWrite(char c) {
    BT.write(c);
}

void btWriteString(const char* s) {
    BT.print(s);
}

void btWriteFloat(float f) {
    BT.print(f, 2);     // 保留 2 位小数
}
