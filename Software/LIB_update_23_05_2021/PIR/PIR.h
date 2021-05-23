#ifndef __PIR_H
#define __PIR_H

#include "stm32f4xx_hal.h"

/********************************* PIR Define port connect ********************************/
#define 	PIR_GPIO_PORT 	GPIOE
#define     PIR_PIN         GPIO_PIN_5

uint8_t read_PIR(void);
#endif 	
