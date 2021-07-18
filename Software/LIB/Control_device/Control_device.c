#include "Control_device.h"
#include "Servo.h"

/*
servo cua so 	0 	open
							70 	close
							
							LED_onboard PA12: phat hien data gui ve
							LED_onboard PA13: kiem tra ket noi wifi
							LED_onboard PA14 + PA15: kiem tra xem phai o trong che do config hay khong 
*/

void control_Fan(int dir, int state){
    if (dir == 1){
        //control FAN 1 at PC2 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, state);					
    }
    else if (dir == 2){
        //control FAN 2 at PC0
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, state);		
    }
}

void control_Led(int dir, int state){
    if (dir == 1){
        //control LED 1 at PA6 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state);					
    }
    else if (dir == 2){
        //control LED 2 at PA4
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state);		
    }
    else if (dir == 3){
        //control LED 3 at PA2 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state);					
    }
    else if (dir == 4){
        //control LED 4 at PA0
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, state);		
    }
}

void control_Relay(int state){
    //control RELAY at PC4
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, state);   
}

void control_Gate(int state){
	if (state == 0){
			//close 
			servo_position(1, 0);
			HAL_Delay(10);
	}
	else if (state == 1){
			//open
			servo_position(1, 70);
			HAL_Delay(10);
	}
}

void control_Pole(int state){
		if (state == 0){
			//close 
			servo_position(2, 0);
			HAL_Delay(10);
		}
		else if (state == 1){
			//open
			servo_position(2, 70);
			HAL_Delay(10);
		}
}

void control_Window(int state){
    if (state == 0){
			//close 
			servo_position(3, 0);
			HAL_Delay(10);
		}
		else if (state == 1){
			//open
			servo_position(3, 70);
			HAL_Delay(10);
		}
}

void control_led_onboard(int dir){
	if (dir == 0){
			//led PD14 kiem tra
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	}
	else if	(dir == 1){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	}
	else if (dir == 2){
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	}
}