#include "PID.h"
#include "Imu.h"
#include "Bt.h"
#include <math.h>
#include "Encoder.h"
#include "Ultrasonic.h"
#define ANGLE_DIR   (+1)
#define GYRO_DIR    (+1)
#define PWM_DIR     (+1)
//pid parameter
static float Kp_angle = 7.0f;
static float Kd_angle = 0.15f;
static float Kp_speed = 5.0f;//0.1f;
static float Ki_speed = Kp_speed/200;//0.005f;
// IMU
static float AngleX, AccX, GyX;
static float AngleZ, AccZ, GyZ;
static float Balance_Angle = -0.3f;

static float gyro_f = 0.0f, angle_f = 0.0f;
static float beta = 0.7f;

static long Target_speed = 0;

static float Encoder = 0.0f;
static float Encoder_Integral = 0.0f;
static int Velocity_Pwm = 0;

//turning
static float Kp_turn = 0.5f;
static float Kd_turn = -0.1f;
static float turn_cmd = 0.0f;
static const float TURN_CMD = 40.0f;
static const float TURN_PWM_MAX = 100.0f; 

//avoidance
static float d_cm = 0;
static const float D_SLOW  = 30.0f;
static const float D_STOP  = 20.0f;
static const float D_BACK  = 12.0f;
static const float D_CRASH = 6.0f;
static volatile bool crashLatched = false;

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

static inline int turn_pd(float gyro_z)
{

    float out = (-Kp_turn * turn_cmd) + (Kd_turn * gyro_z);

    //limit
    out = constrain(out, -TURN_PWM_MAX, TURN_PWM_MAX);
    return (int)out;
}

void PID_PWMcount(int *pwmL, int *pwmR)
{

    if (crashLatched) {
        Encoder_Integral = 0.0f;
        Velocity_Pwm = 0;
        *pwmL = 0;
        *pwmR = 0;
        return;
    }
    get_dataX(AngleX, AccX, GyX);
    get_dataZ(AngleZ, AccZ, GyZ);

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
    int TurnPWM = turn_pd(GyZ);

    if (fabsf(angle_f) > 50.0f) {
        Encoder_Integral = 0.0f;
        Velocity_Pwm = 0;
        *pwmL = 0;
        *pwmR = 0;
        return;
    }
    // Serial.println(Velocity_Pwm);
    int left  = Motor + TurnPWM;
    int right = Motor - TurnPWM;

    left  = PWM_DIR * left;
    right = PWM_DIR * right;

    left  = constrain(left,  -255, 255);
    right = constrain(right, -255, 255);

    *pwmL = (int)left;
    *pwmR = (int)right;
}

void Bluetooth_Debug(){
    while (btAvailable()){
        char Receive = btRead();
        // Serial.println(Balance_Angle);
        // Serial.println(Receive);
        if (Receive == '\n' || Receive == '\r') continue;
        switch (Receive){
            case '1': Balance_Angle += 0.1;break;
            case '2': Balance_Angle -= 0.1;break;
            case '3':Kp_speed += 0.05 ;break;
            case '4':Kp_speed -= 0.05 ;break;
            case '5':Ki_speed += 0.05 ;break;
            case '6':Ki_speed -= 0.05 ;break;
            case '7': Kd_angle += 0.02;break;
            case '8': Kd_angle -= 0.02;break;
            case 'S': Balance_Angle = -0.3, turn_cmd = 0.0f;break;
            case 'F': Balance_Angle = -7;break;
            case 'B': Balance_Angle = 7;break;
            case 'L': turn_cmd = -TURN_CMD;break;
            case 'R': turn_cmd = +TURN_CMD;break;
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

    btWriteString(";DIST:");
    btWriteFloat(d_cm);

    btWriteString(";\n");        
}

void updateAvoidanceTargetAngle()
{
    d_cm = GetDistance();
    if (crashLatched) return;

    if (d_cm <= 0) {
    return;
    }
  
    if (d_cm > D_SLOW) {
    return;
    }

    // 2) SLOW DOWN: D_STOP < d <= D_SLOW
    if (d_cm > D_STOP && d_cm <= D_SLOW) {
    float k = (d_cm - D_STOP) / (D_SLOW - D_STOP);
    if (k < 0) k = 0;
    if (k > 1) k = 1;

    Balance_Angle = -0.3+(Balance_Angle*k);
    return;
    }

  // 3) STOP: D_BACK < d <= D_STOP
    if (d_cm > D_BACK && d_cm <= D_STOP) {
      Balance_Angle = -0.3;
      return;
    }

  // 4) BACK: d <= D_BACK
    if (d_cm <= D_BACK && d_cm > 0) {
        Balance_Angle = 7;
     }
  
    if (d_cm <= D_CRASH && d_cm > 0) {
         crashLatched = true;
         Balance_Angle = -0.3;

        // 清积分，避免解锁瞬间冲一下
         Encoder_Integral = 0.0f;
        Velocity_Pwm = 0;
        return;
    }
      return;
}
