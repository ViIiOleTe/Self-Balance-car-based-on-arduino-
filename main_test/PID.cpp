#include "PID.h"
#include "Imu.h"
#include "Bt.h"
#include <math.h>
#include "Encoder.h"

#define ANGLE_DIR   (+1)
#define GYRO_DIR    (+1)
#define PWM_DIR     (+1)

// ===== smoothing & limits =====
static const float dt_ctrl = 0.005f;   // 5ms
static float gyro_lp = 0.0f;           // filtered gyro for D
static float angle_lp = 0.0f;          // filtered angle
static const float a_angle = 0.85f;    // angle LPF (0.8~0.95)
static const float a_gyro  = 0.7f;     // gyro LPF  (0.6~0.85)

// speed PI behavior like “平衡小车之家”
static const float ENC_LPF_A = 0.7f;   // like their Encoder=0.7*Encoder+0.3*Least
static const float ENC_LPF_B = 0.3f;
static const float I_LIM = 8000.0f;    // try 3000~21000
static const float I_RELEASE_THRESH = 300.0f;
static const float I_RELEASE_STEP   = 200.0f;

// pwm limit like theirs
static const int PWM_MAX = 250;

// motor deadband compensation (needs tuning)
static const int PWM_DEADBAND = 0;    // try 15~35

//pid parameter
static float Kp_angle = -7;
static float Kd_angle = -0.2;
static float Kp_speed = -1.5;//-3.0f;
static float Ki_speed = Kp_speed/200;//0.005f;
// IMU
static float AngleX, AccX, GyX;
static float AngleZ, AccZ, GyZ;
static float Balance_Angle = -2.3f;

static float gyro_f = 0.0f, angle_f = 0.0f;
static float beta = 0.7f;

static long Target_speed = 0;

static float Encoder = 0.0f;
static float Encoder_Integral = 0.0f;
static int Velocity_Pwm = 0;

//turning
static float Kp_turn = 0;
static float Kd_turn = 0;
static float turn_cmd = 0.0f;
static const float TURN_CMD = 40.0f;
static const float TURN_PWM_MAX = 100.0f; 

static inline int balance_pd(float angle_meas, float gyro_meas)
{
    float Bias = angle_meas - Balance_Angle;
    float out  = Kp_angle * Bias + Kd_angle * gyro_meas;
    return (int)out;
}

static inline int velocity_pi_update(long speed_pulses_40ms, long target_pulses_40ms)
{
    float Encoder_Least = (float)(speed_pulses_40ms - target_pulses_40ms);

    // 1st-order LPF like their code
    // Encoder = ENC_LPF_A * Encoder + ENC_LPF_B * Encoder_Least;
    Encoder = Encoder_Least;
    // integral (40ms step)
    Encoder_Integral += Encoder;

    // // if target is zero, quickly discharge integral (prevents standstill dithering)
    // if (target_pulses_40ms == 0) {
    //     if (Encoder_Integral > I_RELEASE_THRESH)  Encoder_Integral -= I_RELEASE_STEP;
    //     if (Encoder_Integral < -I_RELEASE_THRESH) Encoder_Integral += I_RELEASE_STEP;
    // }

    // integral limit (wider than ±300)
    Encoder_Integral = constrain(Encoder_Integral, -I_LIM, I_LIM);

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

// static inline int apply_deadband(int pwm)
// {
//     if (pwm == 0) return 0;
//     if (pwm > 0)  return (pwm < PWM_DEADBAND) ? PWM_DEADBAND : pwm;
//     else          return (pwm > -PWM_DEADBAND) ? -PWM_DEADBAND : pwm;
// }

void PID_PWMcount(int *pwmL, int *pwmR)
{
    get_dataX(AngleX, AccX, GyX);
    get_dataZ(AngleZ, AccZ, GyZ);

    float gyro_raw  = (float)GYRO_DIR  * GyX;
    float angle_raw = (float)ANGLE_DIR * AngleX;

    angle_lp = a_angle * angle_lp + (1.0f - a_angle) * angle_raw;
    // gyro_lp  = a_gyro  * gyro_lp  + (1.0f - a_gyro ) * gyro_raw;

    angle_f = angle_lp;
    gyro_f  = gyro_raw;

    int Balance_Pwm = balance_pd(angle_f, gyro_f);

    long sumPulses40ms = 0;
    if (encoderSampleSum(sumPulses40ms, 40)) {
        Velocity_Pwm = velocity_pi_update(sumPulses40ms, Target_speed);
    }

    int Motor = Balance_Pwm - Velocity_Pwm;
    int TurnPWM = turn_pd(GyZ);

    if (fabsf(angle_f) > 40.0f) {
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

    left  = constrain(left,  -PWM_MAX, PWM_MAX);
    right = constrain(right, -PWM_MAX, PWM_MAX);

    // // compensate deadband (helps stop micro-oscillation)
    // left  = apply_deadband(left);
    // right = apply_deadband(right);

   
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

    btWriteString(";\n");        
}




