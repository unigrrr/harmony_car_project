#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "sys.h"

#define AIN     PBout(13)  
#define BIN     PBout(14)    

#define PWMA    TIM4->CCR1     
#define PWMB    TIM4->CCR2      

void Motor_Init(void);
void PWM_Init(u16 arr, u16 psc);
void Set_Pwm(int moto1, int moto2);
u32 myabs(long int a);

#endif

