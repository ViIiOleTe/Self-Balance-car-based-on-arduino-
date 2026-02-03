#include "PID.h"
#include "Imu.h"
#include "Bt.h"
#include <math.h>
#include "Encoder.h"

#define ANGLE_DIR   (+1)
#define GYRO_DIR    (+1)
#define PWM_DIR     (+1)

static float Kp_angle = 7.0f;
static float Kd_angle = 0.15f;
static float Kp_speed = 5.0f;//0.1f;
static float Ki_speed = Kp_speed/200;//0.005f;
// IMU
static float AngleX, AccX, GyX;
static float Balance_Angle = -0.3f;

static float gyro_f = 0.0f, angle_f = 0.0f;
static float beta = 0.7f;

static long Target_speed = 0;

static float Encoder = 0.0f;
static float Encoder_Integral = 0.0f;
static int Velocity_Pwm = 0;


static inline int balance_pd(float angle_meas, float gyro_meas)
{
    float Bias = angle_meas - Balance_Angle;
    float out  = Kp_angle * Bias + Kd_angle * gyro_meas;
    return (int)out;
}

static inline int velocity_pi_update(long speed_pulses_40ms, long target_pulses_40ms)
{
    // Encoder_Least = meas - target
    float Encoder_Least = (float)(speed_pulses_40ms - target_pulses_40ms);
    Encoder = 0.5f * Encoder + 0.5f * Encoder_Least;
    Encoder_Integral += Encoder;

    // intigral limitation
    Encoder_Integral = constrain(Encoder_Integral, -300.0f, 300.0f);

    float out = Kp_speed * Encoder + Ki_speed * Encoder_Integral;
    return (int)out;
}

int16_t PID_PWMcount()
{
    get_dataX(AngleX, AccX, GyX);

    float gyro_raw  = (float)GYRO_DIR  * GyX;
    float angle_raw = (float)ANGLE_DIR * AngleX;

    gyro_f  = gyro_raw;
    angle_f = beta  * angle_f + (1.0f - beta)  * angle_raw;

    int Balance_Pwm = balance_pd(angle_f, gyro_f);

    long sumPulses40ms = 0;
    if (encoderSampleSum(sumPulses40ms, 40)) {
        Velocity_Pwm = velocity_pi_update(sumPulses40ms, Target_speed);
    }

    int Motor = Balance_Pwm - Velocity_Pwm;

    if (fabsf(angle_f) > 50.0f) {
        Encoder_Integral = 0.0f;
        Velocity_Pwm = 0;
        return 0;
    }
    // Serial.println(Velocity_Pwm);
    Motor = PWM_DIR * Motor;
    Motor = constrain(Motor, -255, 255);
    return (int16_t)Motor;
}

void Bluetooth_Debug(){
    while (btAvailable()){
        char Receive = btRead();
        // Serial.println(Balance_Angle);
        // Serial.println(Receive);
        switch (Receive){
            case '1': Balance_Angle += 0.1;break;
            case '2': Balance_Angle -= 0.1;break;
            case '3':Kp_speed += 0.05 ;break;
            case '4':Kp_speed -= 0.05 ;break;
            case '5':Ki_speed += 0.05 ;break;
            case '6':Ki_speed -= 0.05 ;break;
            case '7': Kd_angle += 0.02;break;
            case '8': Kd_angle -= 0.02;break;
            case 'S': Balance_Angle = -0.3;break;
            case 'F': Balance_Angle += 5;break;
            case 'B': Balance_Angle -= 5;break;
            case '9': sendTelemetry();break;
        }

    }
}

void sendTelemetry() {
    // send datat to app
    btWriteString("ANGLE:");
    btWriteFloat(AngleX);         

    btWriteString(";SPEED:");
    btWriteFloat(Velocity_Pwm);            

    btWriteString(";KP:");
    btWriteFloat(Kp_speed);

    btWriteString(";KI:");
    btWriteFloat(Ki_speed);

    btWriteString(";KD:");
    btWriteFloat(Kd_angle);

    btWriteString(";BalAng:");
    btWriteFloat(Balance_Angle);

    btWriteString(";\n");        
}




