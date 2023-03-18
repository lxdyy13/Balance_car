#ifndef _MOTOR_H
#define _MOTOR_H
#include "main.h"
#include "gpio.h"

void Motor_Start(void);
void Motor_Rotaton(int MotorA,int MotorB);
int GFP_abs(int a);
void Limit(int *motoA,int *motoB);
void Stop(float angle, float voltage);

void Encoder_Start(void);    //����������
int Read_Speed(int TIMx);   //�ӱ�������ȡ���ת������

#endif

