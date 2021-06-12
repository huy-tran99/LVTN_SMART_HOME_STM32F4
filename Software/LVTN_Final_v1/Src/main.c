/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "dht11.h"
#include "GAS.h"
#include "lcd_16x2.h"
#include "lcd_20x4.h"
#include "Light_sensor.h"
#include "music.h"
#include "PIR.h"
#include "Rain.h"
#include "Servo.h"
#include "finger.h"
#include "lib_keypad.h"
#include "Control_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

osThreadId Check_wifiHandle;
osThreadId Finger_printHandle;
osThreadId Data_processingHandle;
osThreadId Smart_controlHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
void Check_wifiTask(void const * argument);
void Finger_printTask(void const * argument);
void Data_processingTask(void const * argument);
void Smart_controlTask(void const * argument);

/* USER CODE BEGIN PFP */
struct Data_read{
		unsigned char dht11[4];
		unsigned char dht11_old[4];
		uint16_t gas_old;
}data_read;

struct Data_send{
		char DHT11[17];
		char GAS[14];
		char LED1[8];
		char LED2[8];
		char LED3[8];
		char LED4[8];
		char SECURITY[12];
		char RELAY[9];
		char FAN1[8];
		char FAN2[8];
		char ERASE[9]; //to erase flash EEPROM
}data_send;


struct Firebase{
		int FAN1_state;
		int FAN2_state;
		int RELAY_state;
		int LED1_state;
		int LED2_state;
		int LED3_state;
		int LED4_state;
		int DOOR_state;
}FB;

struct Device{
		int FAN1_state;
		int FAN2_state;
		int RELAY_state;
		int LED1_state;
		int LED2_state;
		int LED3_state;
		int LED4_state;
		int DOOR_state;
		int FAN1_old_state;
		int FAN2_old_state;
		int RELAY_old_state;
		int LED1_old_state;
		int LED2_old_state;
		int LED3_old_state;
		int LED4_old_state;
		int DOOR_old_state;
}DV;

struct Touch{
		int state1;
		int state2;
		int state3;
}touch;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Code for comunication with esp8266 using UART 1 and UART 6 */
/*	define */
#define txBuf_size 30
#define rxBuf_size 10

char tx_Buf[txBuf_size];
char rx_Buf[rxBuf_size];

char rxBuf_receive[2];
uint16_t rx_Index = 0;
char temp_receive[rxBuf_size]; //data temp read from esp

/* Function to comunication */
static void serial_write(void* data, size_t len){
	HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, 100);
}

void serial_send_cmd(char data[]){
		char line[txBuf_size];
		snprintf(line, sizeof(line), "%s\r\n", data);
		serial_write(line, strlen(line));
}

char* serial_read(){
	strcpy(temp_receive, rx_Buf);
	return temp_receive;
}

/* function to check connect wifi */
int check_wifi(){
	char temp[2];
	if (strncmp(serial_read(),"wifi",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			return 0; //disconnected
		}
		if (temp[0] == '1'){
			return 1; //connected
		}
		if (temp[0] == '2'){
			return 2; //smartconfig
		}
	}
}

/* Function to read sensor */
void readSensor(){
		read_DHT11(data_read.dht11);
		if ((data_read.dht11_old[0] != data_read.dht11[0])||(data_read.dht11_old[2] != data_read.dht11[2])||(data_read.dht11_old[1] != data_read.dht11[1])||(data_read.dht11_old[3] != data_read.dht11[3])){
			memcpy(data_read.dht11_old, data_read.dht11, 4);
			//meaning data change 
			sprintf(data_send.DHT11, "DHT11 %d.%d %d.%d\r\n" ,data_read.dht11_old[2],data_read.dht11_old[3],data_read.dht11_old[0],data_read.dht11_old[1]);
			serial_write(data_send.DHT11, strlen(data_send.DHT11));
			HAL_Delay(500);
		}
		
		uint16_t c = 0; //do thay doi cua cam bien gas
		c = (data_read.gas_old > read_GAS()) ? data_read.gas_old - read_GAS(): read_GAS() - data_read.gas_old;
		if (c > 0){
			data_read.gas_old = read_GAS();
			sprintf(data_send.GAS, "GAS %d\r\n" ,data_read.gas_old);
			serial_write(data_send.GAS, strlen(data_send.GAS));
			HAL_Delay(500);
		}	
}

/* stream Firebase */
void stream(){
	char temp[2];
	//stream FAN state
	if (strncmp(serial_read(),"FAN1",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.FAN1_state = 0;
		}
		if (temp[0] == '1'){
			FB.FAN1_state = 1;
		}
	}
	if (strncmp(serial_read(),"FAN2",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.FAN2_state = 0;
		}
		if (temp[0] == '1'){
			FB.FAN2_state = 1;
		}
	}
	//stream RELAY state
	if (strncmp(serial_read(),"RELAY",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.RELAY_state = 0;
		}
		if (temp[0] == '1'){
			FB.RELAY_state = 1;
		}
	}
	//stream LED1 state
	if (strncmp(serial_read(),"LED1",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED1_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED1_state = 1;
		}
	}
	//stream LED2 state 
	if (strncmp(serial_read(),"LED2",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED2_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED2_state = 1;
		}
	}	
	//stream lED3 state
	if (strncmp(serial_read(),"LED3",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED3_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED3_state = 1;
		}
	}	
	//stream LED4 state
	if (strncmp(serial_read(),"LED4",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED4_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED4_state = 1;
		}
	}	
	//stream DOOR state
	if (strncmp(serial_read(),"DOOR",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.DOOR_state = 0;
		}
		if (temp[0] == '1'){
			FB.DOOR_state = 1;
		}
	}	
}

void check_data_stream(){
		if (DV.FAN1_state != FB.FAN1_state){
			DV.FAN1_state = FB.FAN1_state;
		}		
		if (DV.FAN2_state != FB.FAN2_state){
			DV.FAN2_state = FB.FAN2_state;
		}			
		if (DV.RELAY_state != FB.RELAY_state){
			DV.RELAY_state = FB.RELAY_state;
		}	
		if (DV.LED1_state != FB.LED1_state){
			DV.LED1_state = FB.LED1_state;
		}	
		if (DV.LED2_state != FB.LED2_state){
			DV.LED2_state = FB.LED2_state;
		}	
		if (DV.LED3_state != FB.LED3_state){
			DV.LED3_state = FB.LED3_state;
		}	
		if (DV.LED4_state != FB.LED4_state){
			DV.LED4_state = FB.LED4_state;
		}	
		if (DV.DOOR_state != FB.DOOR_state){
			DV.DOOR_state = FB.DOOR_state;
		}	
}




void send_stream(){
	if (DV.LED1_state != DV.LED1_old_state){
		DV.LED1_old_state = DV.LED1_state;
		sprintf(data_send.LED1, "LED1 %d\r\n" ,DV.LED1_old_state);
		serial_write(data_send.LED1, strlen(data_send.LED1));
		HAL_Delay(200);
	}
	if (DV.LED2_state != DV.LED2_old_state){
		DV.LED2_old_state = DV.LED2_state;
		sprintf(data_send.LED2, "LED2 %d\r\n" ,DV.LED2_old_state);
		serial_write(data_send.LED2, strlen(data_send.LED2));
		HAL_Delay(200);
	}
	
	if (DV.LED3_state != DV.LED3_old_state){
		DV.LED3_old_state = DV.LED3_state;
		sprintf(data_send.LED3, "LED3 %d\r\n" ,DV.LED3_old_state);
		serial_write(data_send.LED3, strlen(data_send.LED3));
		HAL_Delay(200);
	}
	if (DV.LED4_state != DV.LED4_old_state){
		DV.LED4_old_state = DV.LED4_state;
		sprintf(data_send.LED4, "LED4 %d\r\n" ,DV.LED4_old_state);
		serial_write(data_send.LED4, strlen(data_send.LED4));
		HAL_Delay(200);
	}
	if (DV.RELAY_state != DV.RELAY_old_state){
		DV.RELAY_old_state = DV.RELAY_state;
		sprintf(data_send.RELAY, "RELAY %d\r\n" ,DV.RELAY_old_state);
		serial_write(data_send.RELAY, strlen(data_send.RELAY));
		HAL_Delay(200);
	}
	if (DV.FAN1_state != DV.FAN1_old_state){
		DV.FAN1_old_state = DV.FAN1_state;
		sprintf(data_send.FAN1, "FAN1 %d\r\n" ,DV.FAN1_old_state);
		serial_write(data_send.FAN1, strlen(data_send.FAN1));
		HAL_Delay(200);
	}
	if (DV.FAN2_state != DV.FAN2_old_state){
		DV.FAN2_old_state = DV.FAN2_state;
		sprintf(data_send.FAN2, "FAN2 %d\r\n" ,DV.FAN2_old_state);
		serial_write(data_send.FAN2, strlen(data_send.FAN2));
		HAL_Delay(200);
	}
	if (DV.DOOR_state != DV.DOOR_old_state){
		DV.DOOR_old_state = DV.DOOR_state;
		sprintf(data_send.SECURITY, "SECURITY %d\r\n" ,DV.DOOR_old_state);
		serial_write(data_send.SECURITY, strlen(data_send.SECURITY));
		HAL_Delay(200);
	}
}

void lcd_display(){
	
}

void read_button_all(){
	
}

void smart_control(){
	
}

/* Code for comunication with R305 using UART 3*/
int p = -1;
uint8_t txBuf[32];
uint8_t rxBuf[32];
uint8_t rxData;
uint8_t rxIndex = 0;
//uint16_t RxLastTime = 0;
uint8_t PageID = 122;
uint8_t test = 0, key = 0;

/* Receive using ISR */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Prevent unused argument(s) compilation warning */
	if(UartHandle->Instance==USART6){
		if (rx_Index == 0){
			for(int i = 0; i < rxBuf_size; i++){
				rx_Buf[i] = 0;
			}
		}
		if (rxBuf_receive[0] != 10){
			rx_Buf[rx_Index]=rxBuf_receive[0];
			rx_Index++;
		}
		else{
				rx_Index = 0;
		}
		HAL_UART_Receive_IT(&huart6 , (uint8_t*)rxBuf_receive, 1);
	}
	
	if(UartHandle->Instance==USART3){
		rxBuf[rxIndex] = rxData;	
		if (rxIndex < (sizeof(rxBuf)-2))
			rxIndex++;
		HAL_UART_Receive_IT(&huart3, &rxData, 1);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	music_Init();
	servo_Init();
	DHT11_Init();
	
	LCD_16x2_i2cDeviceCheck();
	LCD_16x2_Init();
	LCD_16x2_BackLight(LCD_BL_ON);
	LCD_16x2_SetCursor(1,1);
	LCD_16x2_Send_String("Hello",STR_NOSLIDE);
	
	LCD_20x4_i2cDeviceCheck();
	LCD_20x4_Init();
	LCD_20x4_BackLight(LCD_BL_ON);
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
	LCD_20x4_SetCursor(2,1);
	LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
	
	
	/* Begin receive data from uart 3 and uart 6*/
	HAL_UART_Receive_IT(&huart6 , (uint8_t*)rxBuf_receive, 1);
	HAL_UART_Receive_IT(&huart3, &rxData, 1);
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Check_wifi */
  osThreadDef(Check_wifi, Check_wifiTask, osPriorityHigh, 0, 128);
  Check_wifiHandle = osThreadCreate(osThread(Check_wifi), NULL);

  /* definition and creation of Finger_print */
  osThreadDef(Finger_print, Finger_printTask, osPriorityNormal, 0, 128);
  Finger_printHandle = osThreadCreate(osThread(Finger_print), NULL);

  /* definition and creation of Data_processing */
  osThreadDef(Data_processing, Data_processingTask, osPriorityRealtime, 0, 128);
  Data_processingHandle = osThreadCreate(osThread(Data_processing), NULL);

  /* definition and creation of Smart_control */
  osThreadDef(Smart_control, Smart_controlTask, osPriorityAboveNormal, 0, 128);
  Smart_controlHandle = osThreadCreate(osThread(Smart_control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 900-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 900-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 900-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC2 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA4 PA5 
                           PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Check_wifiTask */
/**
  * @brief  Function implementing the Check_wifi thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Check_wifiTask */
void Check_wifiTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_Finger_printTask */
/**
* @brief Function implementing the Finger_print thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Finger_printTask */
void Finger_printTask(void const * argument)
{
  /* USER CODE BEGIN Finger_printTask */
  /* Infinite loop */
  for(;;)
  {		
		Enter();
    osDelay(1);
  }
  /* USER CODE END Finger_printTask */
}

/* USER CODE BEGIN Header_Data_processingTask */
/**
* @brief Function implementing the Data_processing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_processingTask */
void Data_processingTask(void const * argument)
{
  /* USER CODE BEGIN Data_processingTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Data_processingTask */
}

/* USER CODE BEGIN Header_Smart_controlTask */
/**
* @brief Function implementing the Smart_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Smart_controlTask */
void Smart_controlTask(void const * argument)
{
  /* USER CODE BEGIN Smart_controlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Smart_controlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
