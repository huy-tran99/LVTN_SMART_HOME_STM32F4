#include "Rain.h"

uint8_t read_Rain(void){
	if (HAL_GPIO_ReadPin(RAIN_GPIO_PORT, RAIN_PIN)==RESET){
		return 1;
	}
	else{
		return 0;
	}
}

