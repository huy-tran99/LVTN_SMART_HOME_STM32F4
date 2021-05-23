#ifndef __Servo_H
#define __Servo_H

#include "stm32f4xx_hal.h"

/********************************* Function Servo ********************************/
void servo_Init(void);
int map(int st1, int fn1, int st2, int fn2, int value);
void servo_position(int index, int angle);

#endif 	
