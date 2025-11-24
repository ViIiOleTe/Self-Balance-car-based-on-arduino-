#include "PID.h"
#include "Imu.h"
#include "Bt.h"
#include <math.h>         // fabsf

// static float Balance_Angle = 0;
static float Kp = 0.1, Ki = 0.1, Kd = 0.1;
static float AngleX, AccX, GyX;
static float Balance_Angle = 0.0, ANG_intg = 0.0;
int16_t ANG_PWM;

int16_t PID_PWMcount(){
    get_dataX(AngleX, AccX, GyX);

    float erro = AngleX - Balance_Angle; //count error

    ANG_intg += erro; //accumulate error
    ANG_intg = constrain(ANG_intg, -800, 800);

    ANG_PWM = Kp*erro + Ki*ANG_intg + Kd*GyX;

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
            case '3':Kp += 0.1 ;break;
            case '4':Kp -= 0.1 ;break;
            case '5':Ki += 0.1 ;break;
            case '6':Ki -= 0.1 ;break;
            case '7': Kd += 0.1;break;
            case '8': Kd -= 0.1;break;
        }

    }
}
