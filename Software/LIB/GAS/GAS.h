#ifndef __GAS_H
#define __GAS_H

#include "stm32f4xx_hal.h"

/********************************* GAS Define port connect ********************************/
#define 	GAS_ADC 	hadc1
#define 	GAS_thresh 5
//uint16_t read_GAS(void);
//void convert_ppm(void);
float read_gas_ppm();

#endif 	
 