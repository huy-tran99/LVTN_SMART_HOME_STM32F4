#ifndef __Light_sensor_H
#define __Light_sensor_H

#include "stm32f4xx_hal.h"

/********************************* PIR Define port connect ********************************/
#define 		Light_GPIO_PORT 	GPIOA
#define     Light_PIN           GPIO_PIN_3

uint8_t read_Light_sensor(void);
#endif 	
