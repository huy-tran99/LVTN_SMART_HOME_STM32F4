#ifndef __LIB_TA12_H
#define __LIB_TA12_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

/********************************* GAS Define port connect ********************************/
#define 	Current_ADC 	hadc2

typedef struct{
	uint32_t adc;
	uint32_t current;
	uint64_t start_time;
	uint32_t Max;
}TA12_t;

uint8_t read_Current(int threshold);

#endif