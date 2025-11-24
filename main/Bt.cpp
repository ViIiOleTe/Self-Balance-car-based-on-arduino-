#include "Bt.h"
#include <SoftwareSerial.h>

static SoftwareSerial BT(BT_RX, BT_TX);   // RX, TX

void btBegin() {
    BT.begin(9600);
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