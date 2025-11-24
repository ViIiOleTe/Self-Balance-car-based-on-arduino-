
#define AIN1 12
#define AIN2 13
#define BIN1 7
#define BIN2 6
#define PWMA 11
#define PWMB 10
#define Start_PWMA 9//start compensation value A
#define Start_PWMB 10//start compensation value B


int PWM = 0;
int SCA = Start_PWMA;
int SCB = Start_PWMB;

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
      
    }
  }
  Serial.print("PWM:"); Serial.println(PWM);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  serial_Debug();
  Wheel_drive(PWM, SCA, SCB);
}
