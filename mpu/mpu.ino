#include <MPU6050_tockn.h>
#include <wire.h>//I2C

#define SDA_Pin A4
#define SCL_Pin A5

MPU6050 mpu(Wire);

float ax, ay, az;
float A,B,C,D,g,f;
uint32_t lastPrint = 0;

void init_mpu(){
  Wire.begin();
  mpu.begin(); 
  mpu.calcGyroOffsets(true);
}

void get_angle(float &x, float &y, float &z){
  mpu.update();
  x=mpu.getAngleX();
  y=mpu.getAngleY();
  z=mpu.getAngleZ();
}

void get_acc(float &x, float &y, float &z){
  mpu.update();
  x=mpu.getAccX();
  y=mpu.getAccY();
  z=mpu.getAccZ();
}

void get_gyro(float &x, float &y, float &z){
  mpu.update();
  x=mpu.getGyroX();
  y=mpu.getGyroY();
  z=mpu.getGyroZ();
}

void setup() {
  // put your setup code here, to run once:
   Wire.begin();//start i2c
   Serial.begin(9600);//start serial communications, to view on pc
   init_mpu();
}

void loop() {
  // put your main code here, to run repeatedly:
  // get_angle(ax, f,g);
  // get_acc(A,ay,B);
  // get_gyro(C,D,az);
  mpu.update();

  uint32_t now = millis();
  if (now - lastPrint >= 50) {
    lastPrint = now;
  Serial.print("angleX : ");
  Serial.print(mpu.getAngleX());
  Serial.print("\AccX : ");
  Serial.print(mpu.getAccX());
  Serial.print("\GyroX : ");
  Serial.println(mpu.getGyroX());
  }
  // delay(200);
}
