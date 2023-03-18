#include "motor.h"
#include "tim.h"


void Encoder_Start(){
	__HAL_TIM_SET_COUNTER(&htim2,0);   //�ô��κ����ñ������ĳ�ʼֵΪ0(�漰����ת����Ҫ)
	__HAL_TIM_SET_COUNTER(&htim3,0);

	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);   //�������������ж�ģʽ,������ʱ��ͨ��TI1��TI2��ÿ���������������źŲɼ�ͨ��.
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);  //��������������
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
}

//��ȡ����������ֵ
int Read_Speed(int TIMx)
{
	int Encoder_Value = 0;
	switch(TIMx)
	{
		case 2: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim2);     //�����������������ֵ
			__HAL_TIM_SET_COUNTER(&htim2,0);                   //����֮��Ҫ����,�Ա��´μ�����ȡ.����ÿ����������ֵ��0,ֱ���õ�λʱ��Ļ��Ϳ��Եó��ٶ���Ϣ��.��Ҫ�鷳��Ҫ��ȥ��ֵ��.
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


void Motor_Start(void)     //Motor���������ӵ�PB12~15��ʼ����main()����MX��ִ����.
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //����MX���ú�ԭ��ͼ��������Ҫ��������TIM1��PWM���ͨ��1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //����TIM1��PWM���ͨ��1
}

void Motor_Rotaton(int MotorA,int MotorB)
{
	//1.�о������ţ���Ӧ����ת
	if(MotorA>0)	
	{
		HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
	}//Ain1=1,Ain2=0;//��ת
	else 				
	{
		HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);	
	}//Ain1=0,Ain2=1;//��ת
	//2.�о�PWMֵ
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,GFP_abs(MotorA));   //�Ѵ��ν�����PWMֵ�͸���ʱ��1��channel_1ͨ��.
	
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

/*�޷�����:����PWM���󳬹����Ļ�е����*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)  *motoA=PWM_MAX;
	if(*motoA<PWM_MIN)  *motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)  *motoB=PWM_MAX;
	if(*motoB<PWM_MIN)  *motoB=PWM_MIN;
}

void Stop(float angle, float voltage)
{
		if(angle<-30||angle>30||voltage<11.1)	 //��ص�ѹ����11.1V�رյ��
		{	                                   //===��Ǵ���40�ȹرյ��																			 
				Moto1=0;
				Moto2=0;
		}		
}

