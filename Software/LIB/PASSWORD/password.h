#ifndef __password_H
#define __password_H

#include "stm32f4xx_hal.h"
#include "lib_keypad.h"
#include "finger.h"
#include "lcd_20x4.h"
#include "Servo.h"
#include "music.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

/******************************** Function ********************************/
void change_password(void);
uint8_t get_userfingerid(void);
uint8_t get_userfingerid_delete(void);
int finger_enroll(uint8_t FingerId);
void finger_delete(uint8_t FingerIdDelete);
void verify_password(void);

#endif 	
