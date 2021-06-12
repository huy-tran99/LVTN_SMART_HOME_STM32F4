#include "Control_device.h"

int state = 0;

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

void control_Door(int state){
    //init before
}


int read_Button(int dir){
    //Read pin PE15
    if (dir == 1){
        return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
    }
    //Read pin PB13
    else if (dir == 2){
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    }
    //Read pin PB15
    else if (dir == 3){
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
    }
}

void blink_led_onboard(void){
    //Change state of led on board PA12
    state = !state;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, state);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, state);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, state);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, state);
}

void control_led_onboard(int state){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, state);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, state);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, state);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, state);
}