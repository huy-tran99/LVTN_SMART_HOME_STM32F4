#ifndef __GAS_H
#define __GAS_H

#include "stm32f4xx_hal.h"

/********************************* GAS Define port connect ********************************/
#define 	GAS_ADC 	hadc1

uint8_t read_GAS(uint16_t Thresh);
#endif 	
