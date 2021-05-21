#include "Rain.h"


uint16_t rain_value;

extern ADC_HandleTypeDef Rain_ADC;


uint8_t read_Rain(uint16_t Thresh){
	HAL_ADC_Start (&Rain_ADC);
  	HAL_ADC_PollForConversion (&Rain_ADC, 1000);
	rain_value = HAL_ADC_GetValue(&Rain_ADC);
	if (rain_value < Thresh){
		return rain_value; 
	}
	else{
		return rain_value;
	}
	HAL_ADC_Stop (&Rain_ADC);
}
