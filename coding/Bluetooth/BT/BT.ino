#include <SoftwareSerial.h>
#define Bluetooth_TX 
SoftwareSerial BT(4, 5);  // RX, TX

void setup() {
  Serial.begin(9600);   
  BT.begin(9600);       
  Serial.println("HC-05 Ready");
}

void loop() {
  if (BT.available()) {           
    char c = BT.read();
    Serial.print("BT: ");
    Serial.println(c);
  }

  if (Serial.available()) {       
    char c = Serial.read();
    BT.write(c);                  
  }
}

