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

