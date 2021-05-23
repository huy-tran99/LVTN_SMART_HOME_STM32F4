#include "Light_sensor.h"

uint8_t read_Light_sensor(void){
	if (HAL_GPIO_ReadPin(Light_GPIO_PORT, Light_PIN)==SET){
		return 1; // troi toi
	}
	else{
		return 0;
	}
}
