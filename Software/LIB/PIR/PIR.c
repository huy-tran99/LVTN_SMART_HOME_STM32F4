#include "PIR.h"

uint8_t read_PIR(void){
	if (HAL_GPIO_ReadPin(PIR_GPIO_PORT, PIR_PIN)==RESET){
		return 1;
	}
	else{
		return 0;
	}
}
