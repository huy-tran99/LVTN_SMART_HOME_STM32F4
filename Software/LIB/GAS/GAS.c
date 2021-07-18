#include "GAS.h"
#include "math.h"

//float gas_value;
//double ppm_value;

extern ADC_HandleTypeDef GAS_ADC;

//uint16_t read_GAS(void){
//	HAL_ADC_Start (&GAS_ADC);
//	HAL_ADC_PollForConversion (&GAS_ADC, 100);
//	gas_value = HAL_ADC_GetValue(&GAS_ADC);
//	HAL_ADC_Stop (&GAS_ADC);
//	convert_ppm();
//	return (uint16_t)ppm_value;
//}

//void convert_ppm(void){
//	float R0 = 0.7344;
//	/*Rs volt of air = (1100*5)/4095 
//	 *Rs of air = (5 - Rs volt of air)/Rs volt of air
//	 *R0 = Rs of air / 9.8 
//	*/
//	float gas_volt_value = (gas_value*5)/4095; 
//	float RS = (5 - gas_volt_value)/gas_volt_value; 
//	float Ratio = RS/R0;
//	ppm_value = (log10(Ratio) - 1.29)/(-0.461);
//	ppm_value = pow(10, ppm_value);
//}

float read_gas_ppm(){	
	float gas_value_inVolts = 0.0;
	double ppm_in_log;
	float ppm;
	
	HAL_ADC_Start(&GAS_ADC);
	HAL_Delay(100);
	gas_value_inVolts = HAL_ADC_GetValue(&GAS_ADC);
	HAL_ADC_Stop(&GAS_ADC);
	gas_value_inVolts = 5*gas_value_inVolts/4095;
	
	ppm_in_log = (log10(((5 - gas_value_inVolts)/gas_value_inVolts)/0.1260) - 1.2908)/(-0.461);
	
	ppm = pow(10,ppm_in_log);
	
	return ppm;
}
