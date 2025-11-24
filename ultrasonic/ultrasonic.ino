#define Trig 9
#define Echo 8

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  
  // 10us trigger
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  // read time
  long duration = pulseIn(Echo, HIGH);
  
  // 340m/s = 0.034 cm/us
  float distance = duration * 0.034 / 2.0;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(200);
}
