#include "control_system.h"
//typedef enum {false = 0, true = 1} bool;

/*?? A  B*/
int L_coder, R_coder;

int Motor_A, Motor_B;            //??PWM
int OverflowTime = 100;
volatile uint32_t millis  = 0;   // ????
volatile uint32_t seconds = 0;   // ???

/***************************************************************
??:??PI??
??:??????,?????
??:??PWM
???PID??:
pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)??????
e(k-1)????????  ????
pwm??????
***************************************************************/
int Incremental_PI_A(int Encoders_A, int Target_A)
{
    /* ===== ?????????? ===== */
    float Velocity_KP, Velocity_KI, Velocity_KD;
    static int Pwm_A = 0;
    static int Integral_A = 0;
    static float Error_prev_A = 0;
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;
    float Error_A = (float)(Target_A - Encoders_A);   // ????

    /* ===== ??????????? ===== */
    if (Target_A >= 0) {          // ????(???)
        Velocity_KP = 1.0;
        Velocity_KI = 0.016;
        Velocity_KD = 0.003;
    } else {                      // ????(???????)
        Velocity_KP = 1.2;
        Velocity_KI = 0.016;
        Velocity_KD = 0.003;
    }

    Integral_A += Error_A;   // ????(??)

    // ????
    MaxIntegral = (float)(7199 / Velocity_KI);
    MinIntegral = -(float)(7199 / Velocity_KI);
    if (Integral_A > MaxIntegral) Integral_A = MaxIntegral;
    else if (Integral_A < MinIntegral) Integral_A = MinIntegral;

    Pwm_A += Velocity_KP * Error_A + Velocity_KD * (Error_A - Error_prev_A);

    if (Pwm_A > 7199) Pwm_A = 7199;
    else if (Pwm_A < -7199) Pwm_A = -7199;

    Error_prev_A = Error_A;   // ??????

    return Pwm_A;
}

int Incremental_PI_B(int Encoders_B, int Target_B)
{
    /* ===== ?????????? ===== */
    float Velocity_KP, Velocity_KI, Velocity_KD;
    static int Pwm_B = 0;
    static int Integral_B = 0;
    static float Error_prev_B = 0;
    float MaxIntegral = 0.0;
    float MinIntegral = 0.0;
    float Error_B = (float)(Target_B - Encoders_B);   // ????

    /* ===== ??????????? ===== */
    if (Target_B >= 0) {          // ????(???)
        Velocity_KP = 1.15;
        Velocity_KI = 0.016;
        Velocity_KD = 0.0023;
    } else {                      // ????(???????)
        Velocity_KP = 1.18;
        Velocity_KI = 0.016;
        Velocity_KD = 0.0023;
    }

    Integral_B += Error_B;   // ????(??)

    // ????
    MaxIntegral = (float)(7199 / Velocity_KI);
    MinIntegral = -(float)(7199 / Velocity_KI);
    if (Integral_B > MaxIntegral) Integral_B = MaxIntegral;
    else if (Integral_B < MinIntegral) Integral_B = MinIntegral;

    Pwm_B += Velocity_KP * Error_B + Velocity_KD * (Error_B - Error_prev_B);

    if (Pwm_B > 7199) Pwm_B = 7199;
    else if (Pwm_B < -7199) Pwm_B = -7199;

    Error_prev_B = Error_B;   // ??????

    return Pwm_B;
}

/***************************************************************
??:??????
??:float ?/?
??:int  ???/100ms

??ppr:700,??4
???1?/s,???1???(700*4)??,?100ms?????:
(700*4)/(1000/100),??:??/100ms
***************************************************************/
int Rs_To_CPR(float rads)   // rads??:-1.5 ~ 1.5,?????1.5?/?
{
    int CRP = 0;
    CRP = rads * ((700 * 4) / (1000 / OverflowTime));
    return CRP;
}

/***************************************************************
??:?????
??:?100ms(OverflowTime)?????,????????
     ??3? -> ?1? -> ??3? -> ?1?,????
***************************************************************/
float Get_Target_Speed(void)
{
    static uint32_t ctrl_tick = 0;   // ??????
    uint32_t phase;                  // ????????(??????)

    ctrl_tick++;

    // ?????? 8 ? = 80 ?????(80 x 100ms)
    phase = ctrl_tick % 80;

    if (phase < 30) {
        return 1.0f;    // 0~3 ?:??
    } else if (phase < 40) {
        return 0.0f;    // 3~4 ?:????
    } else if (phase < 70) {
        return -1.0f;   // 4~7 ?:??
    } else {
        return 0.0f;    // 7~8 ?:????
    }
}

/***************************************************************
??:???????
***************************************************************/
void System_Control(void)
{
    int TageA = 0;
    int TageB = 0;
    float target_speed = 0.0f;

    // ???????????(?=??,?=??,0=??)
    target_speed = Get_Target_Speed();

    // ? OverflowTime ms ????????
    L_coder = Read_Encoder(2);
    R_coder = Read_Encoder(3);

    printf("left  coder : %d\r\n", L_coder);
    printf("right coder : %d\r\n", R_coder);

    // ????(?/?)???? OverflowTime ??????
    TageA = Rs_To_CPR(target_speed);
    TageB = Rs_To_CPR(target_speed);

    printf("TageA coder : %d\r\n", TageA);
    printf("TageB coder : %d\r\n", TageB);

    // PI????PWM
    Motor_A = Incremental_PI_A(L_coder, TageA);
    Motor_B = Incremental_PI_B(R_coder, TageB);

    printf("Motor_A pwm : %d\r\n", Motor_A);   // ??:?????? % ?? d
    printf("Motor_B pwm : %d\r\n", Motor_B);

    Set_Pwm(Motor_A, Motor_B);   // ?????
}
