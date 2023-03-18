#include "control.h"
#include "motor.h"

//float balance_UP_KP=480*0.6; 	 // 小车直立环PD参数
//float balance_UP_KD=1.8*0.6;

//float Velocity_Kp=90;     // 小车速度环PI参数
//float Velocity_Ki=90/200;


float balance_UP_KP=290; 	 // 小车直立环PD参数
float balance_UP_KD=1.0;

float Velocity_Kp=90;     // 小车速度环PI参数
float Velocity_Ki=0.6;

float Turn_KP=-30;//转向环KP、KD
float Turn_Kd=-0.7;


int Encoder_Left,Encoder_Right;
 
int PWM_MAX=7200,PWM_MIN=-7200;	//PWM限幅变量
int Moto1=0,Moto2=0;												 //计算出来的最终赋给电机的 PWM

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
float Mechanical_angle=-2.0;         

float Target_Speed=0;
float Turn_Speed=0;

extern int remote_data;
_Bool front=0,back=0,left=0,right=0;

void control_proc(){
	Encoder_Left=-Read_Speed(2);
	Encoder_Right=Read_Speed(3);
	
	rc_deal_proc();
	
	Balance_Pwm = balance_UP(pitch,Mechanical_angle,gyroy);  //===直立环PID控制		
	Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,Target_Speed);       //===速度环PID控制
	Turn_Pwm=Turn_UP(gyroz,Turn_Speed);
	
	Moto1=Balance_Pwm+Velocity_Pwm+Turn_Pwm;
	Moto2=Balance_Pwm+Velocity_Pwm-Turn_Pwm; 
	
	//Moto1=0,Moto2=0;
	Limit(&Moto1,&Moto2);  																					 //===PWM限幅
	Stop(pitch,12);																 //===检查角度以及电压是否正常
	Motor_Rotaton(Moto1,Moto2);                                    //===赋值给PWM寄存器	
}



int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

int velocity(int encoder_left,int encoder_right,int gyro_Z)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral;
   //=============速度PI控制器=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right);//-target;             //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-gyro_Z;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
	  if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===电机关闭后清除积分
	  return Velocity;
}

int Turn_UP(int gyro_Z, int RC)
{
	int PWM_out;
	
	PWM_out=Turn_Kd*gyro_Z + Turn_KP*RC;
	return PWM_out;
}

void rc_deal_proc(){
	int data=remote_data;
	switch(data)
	{
		case 48:
			front=0,back=0,left=0,right=0;break;
		case 49:
			front=1,back=0,left=0,right=0;break;
		case 50:
			front=0,back=1,left=0,right=0;break;
		case 51:
			front=0,back=0,left=1,right=0;break;
		case 52:
			front=0,back=0,left=0,right=1;break;	

		case 10:
			front=0,back=0,left=0,right=0;break;		
	}
	
	if(front==1&&back==0)
		Target_Speed++;
	else if(front==0&&back==1)
		Target_Speed--;
	else
		Target_Speed=0;
	
	if(Target_Speed>100)Target_Speed=100;
	if(Target_Speed<-100)Target_Speed=-100; 
	
	if(left==1&&right==0)//左转                                                           
		Turn_Speed=30;
	else if(left==0&&right==1)//右转
		Turn_Speed=-30;
	else
		Turn_Speed=0;
	
	if(Turn_Speed>50)Turn_Speed=50;
	if(Turn_Speed<-50)Turn_Speed=-50;	
}


