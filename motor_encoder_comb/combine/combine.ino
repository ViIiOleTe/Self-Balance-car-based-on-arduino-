
#define AIN1 12
#define AIN2 13
#define BIN1 7
#define BIN2 6
#define PWMA 11
#define PWMB 10
#define Start_PWMA 9//start compensation value A
#define Start_PWMB 10//start compensation value B
#define ENCA 2// Phase A
#define ENCB 3//Phase B
#define PMA  A0 //Phase B for motor A
#define PMB  A1 //Phase B for motor B

int PWM = 0;
int SCA = Start_PWMA;
int SCB = Start_PWMB;

volatile long pulse_countA = 0;
volatile long pulse_countB = 0;

int lastEncoded = 0; 
float rpmA = 0;
float rpmB = 0;
unsigned long lastTime = 0;

const int pulses_per_rev = 11;    
const float gear_ratio = 89.6; 

//------------------------------Encoder Part----------------------------
//------------------------------Encoder Part----------------------------
void Encoder_init(){
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PMA, INPUT);
  pinMode(PMB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoderB, RISING);
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

void Encoder_Get(){
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


//------------------------------Motor Part----------------------------
//------------------------------Motor Part----------------------------

void Motor_Init(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
}

void serial_Debug(){
  while(Serial.available()){
    char Receive_Char = Serial.read();//use keyboard to test
    switch(Receive_Char){
      case '1' : PWM++; break;
      case '2' : PWM--; break;
      case '0' : PWM = 0; break;
      case '3' : PWM = 245; break;
      case '4' : PWM = -245; break;
    }
  }
  // Serial.print("PWM:"); Serial.println(PWM);
}

void MOTOR_logic(int P1, int P2, int PWM_pin, int pwmx, int SC){
  pwmx = constrain(pwmx, (-255+SC), (255-SC)); //set limit

  if (pwmx == 0){
    digitalWrite(P1, HIGH);
    digitalWrite(P2, HIGH);
    analogWrite(PWM_pin, 0);
  }

  if (pwmx > 0){ //forward rotation
    digitalWrite(P1, HIGH);
    digitalWrite(P2, LOW);
    analogWrite(PWM_pin, pwmx+SC);
  }

   if (pwmx < 0){ //backward rotation
    digitalWrite(P1, LOW);
    digitalWrite(P2, HIGH);
    analogWrite(PWM_pin, -(pwmx-SC));
  }
}

void Wheel_drive(int pwm, int SCA, int SCB){
  MOTOR_logic(AIN1, AIN2, PWMA, pwm, SCA);
  // MOTOR_logic(BIN1, BIN2, PWMB, -pwm);//two wheels are opposite, use negative value here
  MOTOR_logic(BIN1, BIN2, PWMB, pwm, SCB);
}

void setup() {
  Serial.begin(9600);
  Motor_Init();
  Encoder_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  serial_Debug();
  Encoder_Get();
  Wheel_drive(PWM, SCA, SCB);
}