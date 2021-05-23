#include "lib_TA12.h"

int threshold = 50;

extern ADC_HandleTypeDef Current_ADC;
uint32_t TA12_value;

uint32_t current_value;
uint64_t start_time_getTA12Value1=0;
uint32_t read_Current()
{
	if ((HAL_GetTick() - start_time_getTA12Value1) >1000 )
	{		
		start_time_getTA12Value1 = HAL_GetTick();
	
		uint32_t TA12Max = 0;
		uint64_t start_time_getTA12Value2 = HAL_GetTick();
		while((HAL_GetTick()-start_time_getTA12Value2) < 300)//sample for 300ms 
			{ 
				HAL_ADC_Start (&Current_ADC);
				HAL_ADC_PollForConversion (&Current_ADC, 1);
				TA12_value = HAL_ADC_GetValue(&Current_ADC);
				HAL_ADC_Stop (&Current_ADC);
				
				if (TA12_value > TA12Max)
				{
					/*record the maximum sensor value*/
					TA12Max = TA12_value;
				}
			}
		current_value	= TA12Max*1000000/4096*3.3/510/1.414; // mA
	}
	
return (current_value >= threshold);  
	
	
	
	
	
	
}

