#include "PID.h"
#include "Imu.h"
#include "Bt.h"
#include <math.h>         // fabsf
#include "Encoder.h"

// static float Balance_Angle = 0;
static float Kp_angle = 0, Ki_angle = 0, Kd_angle = 0;//kp:7, kd:0.6
static float Kp_speed = 0.2, Ki_speed = 0.01;
static float AngleX, AccX, GyX;
static float Balance_Angle = 0.0, ANG_intg = 0.0;
static float speed_integral = 0;
int16_t ANG_PWM;
static float car_speed = 0;
static float car_rpm = 0;

int16_t PID_PWMcount(){
    //get value from mpu6050
    get_dataX(AngleX, AccX, GyX);
    //get value from encoder
    float rpmA, rpmB;
    encoderSampleRPM(rpmA, rpmB);
    car_rpm = (rpmA + rpmB) * 0.5;

    //speed pid ---------------------------------------------------------
    // car_rpm = (rpmA + rpmB) * 0.5;//average speed
    // car_speed = car_speed * 0.7 + car_rpm * 0.3;//filter

    // float speed_error = 0 - car_speed;
    // speed_integral += speed_error;
    // speed_integral = constrain(speed_integral, -300, 300);
    // //target angle from speed pid
    // float angle_target = Kp_speed * speed_error + Ki_speed * speed_integral;
    //-------------------------------------------------------------------
    //blanced pid------------------------------------------------
    float erro = AngleX - Balance_Angle; //count error
    ANG_intg += erro; //accumulate error
    ANG_intg = constrain(ANG_intg, -800, 800);

    ANG_PWM = Kp_angle*erro + Ki_angle*ANG_intg + Kd_angle*GyX;

    if (fabsf(AngleX) > 45){
        ANG_PWM = 0;
        erro  = 0.0;
        return ANG_PWM;
    }
    if (ANG_PWM > 255){
        ANG_PWM = 255;
    }
    if (ANG_PWM<-255){
        ANG_PWM = -255;
    }
    return ANG_PWM;
}

void Bluetooth_Debug(){
    while (btAvailable()){
        char Receive = btRead();
        // Serial.println(Balance_Angle);
        // Serial.println(Receive);
        switch (Receive){
            case '1': Balance_Angle += 0.1;break;
            case '2': Balance_Angle -= 0.1;break;
            case '3':Kp_angle += 0.5 ;break;
            case '4':Kp_angle -= 0.5 ;break;
            case '5':Ki_angle += 0.05 ;break;
            case '6':Ki_angle -= 0.05 ;break;
            case '7': Kd_angle += 0.05;break;
            case '8': Kd_angle -= 0.05;break;
            case '9': sendTelemetry();break;
        }

    }
}

void sendTelemetry() {
    // send datat to app
    btWriteString("ANGLE:");
    btWriteFloat(AngleX);         

    btWriteString(";SPEED:");
   btWriteFloat(car_rpm);            

    btWriteString(";KP:");
    btWriteFloat(Kp_angle);

    btWriteString(";KI:");
    btWriteFloat(Ki_angle);

    btWriteString(";KD:");
    btWriteFloat(Kd_angle);

    btWriteString(";BalAng:");
    btWriteFloat(Balance_Angle);

    btWriteString(";\n");        
}
