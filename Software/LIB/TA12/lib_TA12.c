#include "lib_TA12.h"
extern ADC_HandleTypeDef Current_ADC;

TA12_t TA12;

uint8_t read_Current(int threshold)
{	
	TA12.Max = 0;
	TA12.start_time = HAL_GetTick();
	while((HAL_GetTick()- TA12.start_time) < 300)//sample for 300ms 
	{ 
		HAL_ADC_Start (&Current_ADC);
		HAL_ADC_PollForConversion (&Current_ADC, 1);
		TA12.adc = HAL_ADC_GetValue(&Current_ADC);
		HAL_ADC_Stop (&Current_ADC);		
		if (TA12.adc > TA12.Max)
		{
			/*record the maximum sensor value*/
			TA12.Max = TA12.adc;
		}
	}
	TA12.current	= TA12.Max*1000000/4096*3.3/510/1.414; // mA
	
	if (TA12.current >= threshold){
		return 1;
	}
	else{
		return 0;
	}
}
