#include "badc.h"

#include "adc.h"
#define N 0.7
double get_volt(void)
{
	unsigned int value;
	static double volt;
	static double last_volt;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
		value= HAL_ADC_GetValue(&hadc1);
	volt=value*36.3/4096;
	volt=N*volt +(1-N)*last_volt;
	last_volt=volt;
	return volt;
}

