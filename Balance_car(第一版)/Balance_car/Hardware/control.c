#include "control.h"
#include "motor.h"

//float balance_UP_KP=480*0.6; 	 // С��ֱ����PD����
//float balance_UP_KD=1.8*0.6;

//float Velocity_Kp=90;     // С���ٶȻ�PI����
//float Velocity_Ki=90/200;


float balance_UP_KP=290; 	 // С��ֱ����PD����
float balance_UP_KD=1.0;

float Velocity_Kp=90;     // С���ٶȻ�PI����
float Velocity_Ki=0.6;

float Turn_KP=-30;//ת��KP��KD
float Turn_Kd=-0.7;


int Encoder_Left,Encoder_Right;
 
int PWM_MAX=7200,PWM_MIN=-7200;	//PWM�޷�����
int Moto1=0,Moto2=0;												 //������������ո�������� PWM

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
	
	Balance_Pwm = balance_UP(pitch,Mechanical_angle,gyroy);  //===ֱ����PID����		
	Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,Target_Speed);       //===�ٶȻ�PID����
	Turn_Pwm=Turn_UP(gyroz,Turn_Speed);
	
	Moto1=Balance_Pwm+Velocity_Pwm+Turn_Pwm;
	Moto2=Balance_Pwm+Velocity_Pwm-Turn_Pwm; 
	
	//Moto1=0,Moto2=0;
	Limit(&Moto1,&Moto2);  																					 //===PWM�޷�
	Stop(pitch,12);																 //===���Ƕ��Լ���ѹ�Ƿ�����
	Motor_Rotaton(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���	
}



int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

int velocity(int encoder_left,int encoder_right,int gyro_Z)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral;
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right);//-target;             //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶ� 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-gyro_Z;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===�����޷�	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
	  if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===����رպ��������
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
	
	if(left==1&&right==0)//��ת                                                           
		Turn_Speed=30;
	else if(left==0&&right==1)//��ת
		Turn_Speed=-30;
	else
		Turn_Speed=0;
	
	if(Turn_Speed>50)Turn_Speed=50;
	if(Turn_Speed<-50)Turn_Speed=-50;	
}


