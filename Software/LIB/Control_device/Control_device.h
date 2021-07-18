#ifndef __CONTROL_DEVICE_H
#define __CONTROL_DEVICE_H

#include "stm32f4xx_hal.h"

void control_Fan(int dir, int state);
void control_Led(int dir, int state);
void control_Relay(int state);
void control_Gate(int state);
void control_Pole(int state);
void control_Window(int state);
void control_led_onboard(int dir);

#endif
