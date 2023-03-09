#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"

extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float Mechanical_angle; 
extern int GFP_abs(int);
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right,int target);
int turn(int encoder_left,int encoder_right,float gyro,int target);

void control_proc(void);
#endif

