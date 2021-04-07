#include "GAS.h"


uint16_t gas_value;

extern ADC_HandleTypeDef GAS_ADC;


uint8_t read_GAS(uint16_t Thresh){
	HAL_ADC_Start (&GAS_ADC);
  HAL_ADC_PollForConversion (&GAS_ADC, 1000);
	gas_value = HAL_ADC_GetValue(&GAS_ADC);
	if (gas_value < Thresh){
		return 1; 
	}
	else{
		return 0;
	}
	HAL_ADC_Stop (&GAS_ADC);
}