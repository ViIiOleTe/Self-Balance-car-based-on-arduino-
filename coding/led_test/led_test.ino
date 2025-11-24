int ledPin = 9;

//让LED亮
// void setup() {
//   pinMode(ledPin, OUTPUT);  // 设置9号口为输出模式
//   digitalWrite(ledPin, HIGH);  // 输出高电平 → LED 亮
// }

// void loop() {
//   // 什么都不用做，LED 会一直亮
// }

// 闪烁LED
void setup(){
  pinMode(ledPin, OUTPUT);
}
// void loop(){
//   digitalWrite(ledPin, HIGH);
//   delay(1000);
//   digitalWrite(ledPin, LOW);
//   delay(1000);
// }

//PWM控制LED呼吸灯
void loop() {
  // 渐亮
  for (int i = 0; i <= 255; i++) {
    analogWrite(ledPin, i); // 输出PWM
    delay(10);
  }
  for (int i = 255; i>0;i--){
    analogWrite(ledPin,i);
    delay(10);
  }
}