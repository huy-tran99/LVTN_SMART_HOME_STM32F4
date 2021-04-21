#ifndef __LIB_KEYPAD_H
#define __LIB_KEYPAD_H
#include "stm32f4xx_hal.h"
#include "lcd_20x4.h"

#define MOCUA HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
#define DONGCUA HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
char read_keypad (void);
void Enter();
void EnterByPassword(void);
void ChangePassword(void);
uint8_t readnumber(void);
void BlockFunction(void);
#endif
