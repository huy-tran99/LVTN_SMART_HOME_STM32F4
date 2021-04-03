#include "lib_GAS_PIR_AS_MUA.h"
#include "main.h"
#ifndef GAS_GPIO_Port
#define GAS_GPIO_Port GPIOA
#define GAS_Pin GPIO_PIN_1
#endif

#ifndef SPEAKER_GPIO_Port
#define SPEAKER_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_2
#endif

#ifndef FAN_GPIO_Port
#define FAN_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_3
#endif

#ifndef PIR_GPIO_Port
#define PIR_GPIO_Port GPIOA
#define PIR_Pin GPIO_PIN_4
#endif

#ifndef ANHSANG_GPIO_Port
#define ANHSANG_GPIO_Port GPIOA
#define ANHSANG_Pin GPIO_PIN_5
#endif

#ifndef DEN_GPIO_Port
#define DEN_GPIO_Port GPIOA
#define DEN_Pin GPIO_PIN_6
#endif

#ifndef MUA_GPIO_Port
#define MUA_GPIO_Port GPIOC
#define MUA_Pin GPIO_PIN_15
#endif

#ifndef XAOPHOIQUANAO_GPIO_Port
#define XAOPHOIQUANAO_GPIO_Port GPIOD
#define XAOPHOIQUANAO_Pin GPIO_PIN_15
#endif


int ReadGas()
{
	if(HAL_GPIO_ReadPin (GAS_GPIO_Port, GAS_Pin)==RESET)									//tin hieu LOW = co Gas
	{
		HAL_GPIO_WritePin (SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);  //Loa keu
		HAL_GPIO_WritePin (FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);			//Quat chaY
		return(1);
	}
	else
	{
		HAL_GPIO_WritePin (SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);  			//Loa tat
		HAL_GPIO_WritePin (FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);				//Quat tat
		return(0);
	}
	
}

int ReadPIR()
{
	if(HAL_GPIO_ReadPin(PIR_GPIO_Port, PIR_Pin)==SET)										//TIN HIEU HIGH = CO CHUYEN DONG
	{
		HAL_GPIO_WritePin (SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);  //Loa keu
		return(1);
	}
	else
	{
		HAL_GPIO_WritePin (SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);  			//Loa tat
		return(0);
	}
}


int ReadAnhSang()
{
	if(HAL_GPIO_ReadPin(ANHSANG_GPIO_Port, ANHSANG_Pin)==SET)						//TIN HIEU HIGH = TROIWF TOIOS = BAT DEN
	{
		HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		return(1);
	}
	else
	{
		HAL_GPIO_WritePin (LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		return(0);
	}
}



int ReadMua()
{
	if(HAL_GPIO_ReadPin(MUA_GPIO_Port, MUA_Pin)==RESET)										//tin hieu LOW = troi mua
	{
		HAL_GPIO_WritePin (XAOPHOIQUANAO_GPIO_Port, XAOPHOIQUANAO_Pin, GPIO_PIN_SET);
		return(1);
	}
	else
	{
		HAL_GPIO_WritePin (XAOPHOIQUANAO_GPIO_Port, XAOPHOIQUANAO_Pin, GPIO_PIN_RESET);
		return(0);
	}
}
