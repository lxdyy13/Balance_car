/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "oled.h"
#include "badc.h"
#include "led.h"

#include "mpu6050.h"
#include "inv_mpu.h"

#include "motor.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char oled_text[16];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double volt=0.0f;

float pitch,roll,yaw; 								  			 //欧拉角(姿态角)
short aacx,aacy,aacz;													 //加速度传感器原始数据
short gyrox,gyroy,gyroz;											 //陀螺仪原始数据 

uint8_t usart3_pData;
uint8_t usart2_pData;
unsigned char buletooth_data[20];

int remote_data=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//定义 串口调试
float tempFloat[3];                    //定义的临时变量
uint8_t tempData[16];                    //定义的传输Buffer
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{ //printf重定向
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return (ch);
}

void oled_show(){
	sprintf(oled_text,"   Balance_car"   );
	OLED_ShowString(0,0,(unsigned char *)oled_text,12);
	
	//OLED_ShowString(0,0,(unsigned char *)"ANGLE:",12);
	OLED_ShowString(0,1,(unsigned char *)"BAT :",12);
	OLED_ShowString(90,1,(unsigned char *)"V",12);
	//OLED_ShowString(0,2,(unsigned char *)"Distance:",12);
	OLED_ShowString(60,2,(unsigned char *)"R: ",12);	
	OLED_ShowString(1,2,(unsigned char *)"L: ",12);
	
	OLED_ShowString(0,4,(unsigned char *)"YAW:",12);
	OLED_ShowString(0,5,(unsigned char *)"PTICH:",12);
	
	OLED_ShowString(0,6,(unsigned char *)"remote: ",12);	
}

void oled_proc(){
	OLED_Float(1,48,volt,1);						//显示电压
	
	OLED_ShowNum(80,2,Encoder_Right,3,12);					//显示右边电机的编码器值
	OLED_ShowNum(20,2,Encoder_Left,3,12);					//显示左边电机的编码器值
	
	OLED_Float(4,56,yaw,1);	//显示yaw
	
	OLED_Float(5,56,pitch,1);	//显示pitch
	
	OLED_ShowNum(60,6,remote_data,3,12);					//显示左边电机的编码器值
	

}
void vofa_send(){
	tempFloat[0] = (float)yaw;    //转成浮点数
	tempFloat[1] = (float)pitch;
	tempFloat[2] = (float)roll;
	memcpy(tempData, (uint8_t *)tempFloat, sizeof(tempFloat));//通过拷贝把数据重新整理
	tempData[12] = 0x00;                    //写如结尾数据
	tempData[13] = 0x00;
	tempData[14] = 0x80;
	tempData[15] = 0x7f;

	HAL_UART_Transmit_IT(&huart1, (uint8_t *)tempData, 16);    //通过串口传输16个数据
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//关闭外部中断
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	//初始化串口
	HAL_UART_Receive_IT(&huart2, &usart2_pData, 1);
	//HAL_UART_Receive_IT(&huart3, &usart3_pData, 1);
	//初始化oled
	SSD1315_Init();	
	HAL_Delay(100);
	OLED_Clear();
	oled_show();
	
	MPU_Init();//=====初始化MPU6050
	HAL_Delay(100);

	//初始化编码器
	Encoder_Start();
	//初始化电机
	Motor_Start();
	
	//初始化mpu
	while(MPU_DMP_Init())//=====初始化MPU6050的DMP模式
	{
		printf("-- DMP Init -- \r\n");
		HAL_Delay(20);
	}
	//初始化控制器
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		volt=get_volt();
		oled_proc();
		//printf("%f,%f,%f\r\n", pitch, roll, yaw);   //串口打印欧拉角
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	switch(GPIO_PIN)
	{
		case GPIO_PIN_5:
		{
			led_proc(50);
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===得到陀螺仪数据
			MPU_DMP_Get_Data(&pitch,&roll,&yaw);										 //===得到欧拉角（姿态角）的数据			
			control_proc();			
		}break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART2)
	{
		remote_data=usart2_pData;
		HAL_UART_Receive_IT(&huart2, &usart2_pData, 1);
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
