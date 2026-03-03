#include "PID.h"
#include "Imu.h"
#include "Bt.h"
#include <math.h>
#include "Encoder.h"
#include "Ultrasonic.h"
#include <KalmanFilter.h>
#define ANGLE_DIR   (+1)
#define GYRO_DIR    (+1)
#define PWM_DIR     (+1)

KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
static int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
//***************下面是卡尔曼滤波相关变量***************//
static float K1 = 0.05; // 对加速度计取值的权重
static float Q_angle = 0.001, Q_gyro = 0.005;
static float R_angle = 0.5 , C_0 = 1;
static float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
static int addr = 0;

//pid parameter
static float Kp_angle = 10;
static float Kd_angle = 0.25;//-0.15;
static float Kp_speed = -2.5; //0.1f;
static float Ki_speed = Kp_speed/200;//0.005f;
// IMU
static float AngleX, AccX, GyX;
static float AngleZ, AccZ, GyZ;
static float Balance_Angle = -0.1;//只有壳子-0.2，有物品-0.8——-1.2

static volatile float gyro_f = 0.0f, angle_f = 0.0f;
static float beta = 0.7f;

static long Target_speed = 0;

static float Encoder = 0.0f;
static float Encoder_Integral = 0.0f;
static int Velocity_Pwm = 0;

//turning
static float Kp_turn = -1;
static float Kd_turn = -0.2;//-0.1f;
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
static bool BACK_FLG;

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
    // 1st-order LPF like their code
    Encoder *= 0.7;
    Encoder += Encoder_Least * 0.3;
    Encoder_Integral += Encoder;

    // 当目标速度为0（停止）时，快速把积分拉回0，避免来回拱
    if (target_pulses_40ms == 0) {
        if (Encoder_Integral > 40.0f)      Encoder_Integral -= 30.0f;
        else if (Encoder_Integral < -40.0f) Encoder_Integral += 30.0f;
    }
    // intigral limitation
    Encoder_Integral = constrain(Encoder_Integral, -8000.0f, 8000.0f);

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

static inline int pwm_small_to_zero(int pwm, int th = 5)
{
    return (pwm >= -th && pwm <= th) ? 0 : pwm;
}

void PID_PWMcount(int *pwmL, int *pwmR)
{

    if (crashLatched) {
        Encoder_Integral = 0.0f;
        Velocity_Pwm = 0;
        *pwmL = 0;
        *pwmR = 0;
    }
    get_kf(ax,ay,az,gx,gy,gz);
    KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); 
    // float gyro_raw  = (float)GYRO_DIR  * GyX;
    // float angle_raw = (float)ANGLE_DIR * AngleX;
    // gyro_f  = gyro_raw;
    // angle_f = beta  * angle_f + (1.0f - beta)  * angle_raw;
    angle_f = KalFilter.angle;
    // gyro_f  = KalFilter.Gyro_x;
    // Serial.println(angle_f);
    
    // g_kf_update_ms  = millis();
    gyro_f  = 0.3*KalFilter.Gyro_x+0.7*gyro_f;
    int Balance_Pwm = balance_pd(KalFilter.angle, KalFilter.Gyro_x);

    long sumPulses40ms = 0;
    if (encoderSampleSum(sumPulses40ms, 40)) {
        Velocity_Pwm = velocity_pi_update(sumPulses40ms, Target_speed);
    }

    int Motor = Balance_Pwm - Velocity_Pwm;
    int TurnPWM = turn_pd(KalFilter.Gyro_z);

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

    left  = constrain(left,  -240, 240);
    right = constrain(right, -240, 240);

    left  = pwm_small_to_zero(left,  2);
    right = pwm_small_to_zero(right, 2);

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
            case 'S': Balance_Angle = -0.1, turn_cmd = 0.0f;break;
            case 'F': Balance_Angle = -5;break;
            case 'B': Balance_Angle = 5;break;
            case 'L': turn_cmd = -TURN_CMD;break;
            case 'R': turn_cmd = +TURN_CMD;break;
            case '9': sendTelemetry();break;
        }

    }
}

void sendTelemetry() {
    // send datat to app
    btWriteString("ANGLE:");
    btWriteFloat(angle_f);  

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
    btWriteFloat(GetDistance());

    btWriteString(";\n");        
}

void updateAvoidanceTargetAngle()
{
    // ===== 参数：只需要 30cm 内停止 =====
    const float STOP_DIST_CM    = 25.0f;   // <= 30cm：禁止前进、强制停
    const float STOP_ANGLE      = -0.1;   // 你的“停止站立角”

    // 1) 读距离（无效则不改变状态）
    float d = GetDistance();
    if (d <= 0) return;   // -1 或 0：本次忽略，避免误触发

    // 2) 维护一个“近距离锁止”状态（滞回）
    static bool nearStopLatched = false;

    if (!nearStopLatched) {
        if (d <= STOP_DIST_CM) nearStopLatched = true;
    } else {
        if (d <= 0) nearStopLatched = false;
    }

           // 3) 自动后退状态机（10cm触发，持续0.5s）
    static bool autoBacking = false;
    static uint32_t autoBackStartMs = 0;

    // 触发：进入10cm以内且当前不在自动后退
    if (!autoBacking && d <= 15) {
        autoBacking = true;
        autoBackStartMs = millis();

        // 清速度环积分，避免动作切换时冲一下
        Encoder_Integral = 0.0f;
        Velocity_Pwm = 0;
    }

    // 如果正在自动后退
    if (autoBacking) {
        // 0.5s 内强制后退角
        if ((uint32_t)(millis() - autoBackStartMs) < 0.5) {
            Balance_Angle = 5;
            return;
        } else {
            // 到时间退出自动后退
            autoBacking = false;
            Balance_Angle = STOP_ANGLE;;
            // 退出时也清一下，避免反弹
            Encoder_Integral = 0.0f;
            Velocity_Pwm = 0;
            // 不 return，继续走下面的“30cm 禁前进”逻辑
        }
    }

    // 3) 不在近距离：完全不干预（允许蓝牙自由控制）
    if (!nearStopLatched) return;

    // 4) 在近距离：只禁止“前进”，允许“后退”
    // ===== 你只需要改这里这一行：怎么判断当前指令是前进？ =====
    // 假设：Balance_Angle 比 STOP_ANGLE 更“前进方向”就算前进
    // 你如果前进方向相反，就把判断符号反过来即可。
    bool isForwardCmd = (Balance_Angle < STOP_ANGLE);   // <-- 如方向不对就改成 (Balance_Angle > STOP_ANGLE)

    if (isForwardCmd) {
        Balance_Angle = STOP_ANGLE;   // 强制停住，禁止前进
        Encoder_Integral = 0.0f;      // 清速度环积分，避免放开后冲一下
        Velocity_Pwm = 0;
    }
    // 如果是后退指令：不改 Balance_Angle，直接放行
}
