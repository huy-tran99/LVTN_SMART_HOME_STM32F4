#ifndef __Rain_H
#define __Rain_H

#include "stm32f4xx_hal.h"

/********************************* Rain Define port connect ********************************/
#define 	Rain_ADC 	hadc1

uint8_t read_Rain(uint16_t Thresh);
#endif 	
