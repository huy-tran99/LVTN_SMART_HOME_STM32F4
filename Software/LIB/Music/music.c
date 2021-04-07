#include "music.h"

/*
	HCLK = 90Mhz
	APB1 and APB2: 45Mhz
	Config frequency 900HZ
*/
/*
	This lib using PWM Timer 9:
	Channel 1: PE5 to send frequency to buzzer
	Value max: 900Hz
*/
void music_Init(void){
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
}


/*
	value from 0 to 900 mean 0Hz to 900Hz 
*/
void music_play(uint16_t index, uint16_t delay_ms){
	if (index < 900){
		htim9.Instance-> CCR1 = index; 
		HAL_Delay(delay_ms);
		htim9.Instance-> CCR1 = 0; 
	}
	else{
		htim9.Instance-> CCR1 = 0; 
	}
}

void music_stop(void){
	htim9.Instance-> CCR1 = 0; 
}