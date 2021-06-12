#ifndef __Rain_H
#define __Rain_H

#include "stm32f4xx_hal.h"

/********************************* Rain Define port connect ********************************/
#define 	RAIN_GPIO_PORT 	 GPIOA
#define     RAIN_PIN         GPIO_PIN_1

uint8_t read_Rain(void);
#endif 	
