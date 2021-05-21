#ifndef __LIB_KEYPAD_H
#define __LIB_KEYPAD_H
#include "stm32f4xx_hal.h"
#include "lcd_20x4.h"

#define MOCUA HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);			
#define DONGCUA HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
 
#define Toggle HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2); 

char read_keypad (void);
void Enter(void);
void EnterByPassword(void);
void ChangePassword(void);
uint8_t readnumber(void);
void BlockFunction(void);
#endif
