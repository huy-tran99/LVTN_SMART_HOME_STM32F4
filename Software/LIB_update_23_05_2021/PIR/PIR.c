#include "PIR.h"

#define PIR_Pin 				GPIO_PIN_7
#define PIR_GPIO_Port 	GPIOA

uint8_t read_PIR(void){
	if (HAL_GPIO_ReadPin(PIR_GPIO_Port, PIR_Pin)==SET){
		return 1; //co chuyen dong
	}
	else{
		return 0;
	}
}
