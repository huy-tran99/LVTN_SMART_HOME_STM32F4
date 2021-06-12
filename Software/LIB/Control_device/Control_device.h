#ifndef __CONTROL_DEVICE_H
#define __CONTROL_DEVICE_H

#include "stm32f4xx_hal.h"

void control_Fan(int dir, int state);
void control_Led(int dir, int state);
void control_Relay(int state);
void control_Door(int state);
int read_Button(int dir);
void blink_led_onboard(void);
void control_led_onboard(int state);

#endif
