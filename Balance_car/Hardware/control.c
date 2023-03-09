#include "control.h"
#include "motor.h"

//float balance_UP_KP=480; 	 // 小车直立环PD参数
//float balance_UP_KD=1.0;
float balance_UP_KP=290; 	 // 小车直立环PD参数
float balance_UP_KD=0.60;

float velocity_KP=40;     // 小车速度环PI参数
float velocity_KI=0.2;

int Encoder_Left,Encoder_Right;

int PWM_MAX=7200,PWM_MIN=-7200;	//PWM限幅变量
int Moto1=0,Moto2=0;												 //计算出来的最终赋给电机的PWM

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
float Mechanical_angle=0.0; 

void control_proc(){
	Encoder_Left=-Read_Speed(2);
	Encoder_Right=Read_Speed(3);
	
	Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===直立环PID控制	
	Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,0);       //===速度环PID控制
	Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz,0);        //===转向环PID控制

//		Moto1=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                 //===计算左轮电机最终PWM
//		Moto2=Balance_Pwm-Velocity_Pwm+Turn_Pwm;                 //===计算右轮电机最终PWM
	Moto1=Balance_Pwm-Velocity_Pwm;
	Moto2=Balance_Pwm-Velocity_Pwm;
	Limit(&Moto1,&Moto2);  																					 //===PWM限幅
	Stop(pitch,12);																 //===检查角度以及电压是否正常
	Motor_Rotaton(Moto1,Moto2);                                    //===赋值给PWM寄存器		

}


int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
	float Bias;
	int balance;
	Bias=Angle-Mechanical_balance;    							 //===求出平衡的角度中值和机械相关
	balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}


int velocity(int encoder_left,int encoder_right,int target)
{  
	static float Velocity,Encoder_Least,Encoder;
	static float Encoder_Integral;
	//=============速度PI控制器=======================//	
	Encoder_Least =(Encoder_Left+Encoder_Right);//-target;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度 
	Encoder *= 0.8;		                                                //===一阶低通滤波器       
	Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-target;                       //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
	if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
	Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===速度控制	
	if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===电机关闭后清除积分
	return Velocity;
}


int turn(int encoder_left,int encoder_right,float gyro,int target)//转向控制
{
	 static float Turn_Target,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=66,Kp=20,Kd=0;
	  //=============遥控左右旋转部分=======================//
	  //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
  	if(target!=0)
		{
			if(++Turn_Count==1)
			Encoder_temp=GFP_abs(encoder_left+encoder_right);      
			Turn_Convert=55/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}
		if(target>0)	         Turn_Target+=Turn_Convert;
		else if(target<0)	     Turn_Target-=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向	速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(target!=0)  Kd=0.5;        
		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		return -Turn_Target*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
}

