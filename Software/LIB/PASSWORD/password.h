#ifndef __password_H
#define __password_H

#include "stm32f4xx_hal.h"
#include "lib_keypad.h"
#include "finger.h"
#include "lcd_20x4.h"
#include "Servo.h"
#include "music.h"
#include "string.h"

/******************************** Function ********************************/
void change_password(void);
int finger_enroll(void);
void verify_password(void);

#endif 	
