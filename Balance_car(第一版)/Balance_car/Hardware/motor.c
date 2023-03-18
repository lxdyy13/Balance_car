#include "motor.h"
#include "tim.h"


void Encoder_Start(){
	__HAL_TIM_SET_COUNTER(&htim2,0);   //用带参宏设置编码器的初始值为0(涉及正反转的需要)
	__HAL_TIM_SET_COUNTER(&htim3,0);

	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);   //开启编码器的中断模式,两个定时器通道TI1和TI2是每个编码器的两个信号采集通道.
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //开启两个编码器
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
}

//获取编码器计数值
int Read_Speed(int TIMx)
{
	int Encoder_Value = 0;
	switch(TIMx)
	{
		case 2: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim2);     //保存编码器计数器的值
			__HAL_TIM_SET_COUNTER(&htim2,0);                   //保存之后要清零,以便下次继续读取.另外每次清零后采样值减0,直接用单位时间的话就可以得出速度信息了.不要麻烦还要减去初值了.
			break;

		case 3: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			break;
		
		default:
			Encoder_Value = 0;
	}	
	return Encoder_Value;
}


void Motor_Start(void)     //Motor控制所连接的PB12~15初始化在main()函数MX中执行了.
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //根据MX配置和原理图：这里需要开启的是TIM1的PWM输出通道1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //开启TIM1的PWM输出通道1
}

void Motor_Rotaton(int MotorA,int MotorB)
{
	//1.研究正负号，对应正反转
	if(MotorA>0)	
	{
		HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
	}//Ain1=1,Ain2=0;//正转
	else 				
	{
		HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);	
	}//Ain1=0,Ain2=1;//反转
	//2.研究PWM值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,GFP_abs(MotorA));   //把传参进来的PWM值送给定时器1的channel_1通道.
	
	if(MotorB>0)
	{
		HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
	}	//	Bin1=1,Bin2=0;
	else
	{
		HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
	}//		Bin1=0,Bin2=1;	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,GFP_abs(MotorB));
}

int GFP_abs(int a)
{ 		   
	int temp;
	temp = a>0?a:(-a);
	return temp;
}

/*限幅函数:避免PWM过大超过马达的机械极限*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)  *motoA=PWM_MAX;
	if(*motoA<PWM_MIN)  *motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)  *motoB=PWM_MAX;
	if(*motoB<PWM_MIN)  *motoB=PWM_MIN;
}

void Stop(float angle, float voltage)
{
		if(angle<-30||angle>30||voltage<11.1)	 //电池电压低于11.1V关闭电机
		{	                                   //===倾角大于40度关闭电机																			 
				Moto1=0;
				Moto2=0;
		}		
}

