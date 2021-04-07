#include "Servo.h"
#include "math.h"

/*
	HCLK = 90Mhz
	APB1 and APB2: 45Mhz
	Config frequency 50HZ
*/
/*
	This lib using PWM Timer 1:
	Channel 1: PE9
	Channel 2: PE11
	Channel 3: PE13 
*/
extern TIM_HandleTypeDef htim1;

void servo_Init(void){
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}

void servo_position(int index, int angle){
	if (index == 1){
		htim1.Instance-> CCR1 = map(0,180,25,125,angle); 
	}
	if (index == 2){
		htim1.Instance-> CCR2 = map(0,180,25,125,angle); 
	}
	if (index == 3){
		htim1.Instance-> CCR3 = map(0,180,25,125,angle); 
	}
}
