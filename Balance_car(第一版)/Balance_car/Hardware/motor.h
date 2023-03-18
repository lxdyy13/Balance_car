#ifndef _MOTOR_H
#define _MOTOR_H
#include "main.h"
#include "gpio.h"

void Motor_Start(void);
void Motor_Rotaton(int MotorA,int MotorB);
int GFP_abs(int a);
void Limit(int *motoA,int *motoB);
void Stop(float angle, float voltage);

void Encoder_Start(void);    //开启编码器
int Read_Speed(int TIMx);   //从编码器读取电机转速数据

#endif

