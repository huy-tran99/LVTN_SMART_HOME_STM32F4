#ifndef __LIB_KEYPAD_H
#define __LIB_KEYPAD_H


#include "stm32f4xx_hal.h"
#include "lcd_20x4.h"
#include "Servo.h"



#define MOCUA 
#define DONGCUA 


char read_keypad (void);
void Enter();
void EnterByPassword(void);
void ChangePassword(void);
uint8_t readnumber(void);
void BlockFunction(void);


#endif


