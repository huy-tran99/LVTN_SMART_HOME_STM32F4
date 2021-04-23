#include "music.h"
#include "math.h"

/*
	HCLK = 90Mhz
	APB1 and APB2: 45Mhz
	Prescaler(PSC - 16 bits value) 450-1
	ARR: 100-1 
	Config frequency: 1000HZ
*/
/*
	This lib using PWM Timer 9:
	Channel 1: PE5 to send frequency to buzzer
	Value will change from 300Hz to 900Hz
*/
extern TIM_HandleTypeDef htim9;

void music_Init(void){
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
}

uint16_t change(uint16_t data){
		return (100000/data);
}

/*
	index from 300 to 900 mean 300 Hz to 900 Hz 
	duty default is 50%
*/
void music_play(uint16_t index){
	if ((index <= 900) && (index >= 300)){
		htim9.Instance-> ARR = change(index); 
		htim9.Instance-> CCR1 = (change(index)/2);
	}
	else{
		htim9.Instance-> CCR1 = 0; 
	}
}

void music_stop(void){
	htim9.Instance-> CCR1 = 0; 
}
