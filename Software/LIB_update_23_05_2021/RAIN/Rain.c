#include "Rain.h"

#define RAIN_Pin 				GPIO_PIN_1
#define RAIN_GPIO_Port 	GPIOA

uint8_t read_Rain()
{
	if (HAL_GPIO_ReadPin(RAIN_GPIO_Port, RAIN_Pin)==RESET)
	{
		return 1; //co mua
	}
	else{
		return 0;
	}
}
