#include "Ultrasonic.h"

void ultra_init() {
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
}

float GetDistance() {
    // trigger 10µs
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);

    // read
    long duration = pulseIn(Echo, HIGH, 30000);   // 30ms overtime

    if (duration == 0) {
        return -1.0; // overtime
    }

    float distance_cm = duration * 0.034f / 2.0f;
    return distance_cm;
}