#ifndef __music_H
#define __music_H

#include "stm32f4xx_hal.h"

/********************************* Function ********************************/
void music_Init(void);
void music_play(uint16_t index);
void music_stop(void);

#endif 	
