#define ENCA 2 // Phase A for motor A
#define ENCB 3 // Phase B for motor B
#define PMA  A0 //Phase B for motor A
#define PMB  A1 //Phase B for motor B
volatile long pulse_countA = 0;
volatile long pulse_countB = 0;

int lastEncoded = 0; 
float rpmA = 0;
float rpmB = 0;
unsigned long lastTime = 0;

const int pulses_per_rev = 11;    
const float gear_ratio = 89.6;   

void setup() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PMA, INPUT);
  pinMode(PMB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, RISING);
  Serial.begin(9600);
}

void Encoder_get(){
  if (millis() - lastTime >= 1000) {  
    noInterrupts();
    long countA = pulse_countA;
    long countB = pulse_countB;
    pulse_countA = 0;
    pulse_countB = 0;
    interrupts();

    rpmA = ((float)countA / (pulses_per_rev * gear_ratio)) * 280;
    rpmB = ((float)countB / (pulses_per_rev * gear_ratio)) * 280;
    Serial.print("SpeedA: ");
    Serial.print(rpmA);
    Serial.println(" RPM");
    lastTime = millis();
  }
}

void loop() {
  Encoder_get();
}

void readEncoderA() {
  int a = digitalRead(ENCA);
  int b = digitalRead(PMA);
  if (a == b) pulse_countA++;   // forward
  else pulse_countA--;         //backward
}

void readEncoderB() {
  int a = digitalRead(ENCB);
  int b = digitalRead(PMB);
  if (a == b) pulse_countB++;   // forward
  else pulse_countB--;         //backward
}

