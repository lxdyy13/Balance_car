#ifndef _MOTOR_H
#define _MOTOR_H
#include "main.h"
void Encoder_Start(void);    //开启编码器
int Read_Speed(int TIMx);   //从编码器读取电机转速数据
#endif

