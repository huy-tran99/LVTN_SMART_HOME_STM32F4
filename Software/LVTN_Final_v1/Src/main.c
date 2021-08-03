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
#include "stdlib.h"
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
#include "lib_TA12.h"
#include "Control_device.h"
#include "password.h"
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

osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
osSemaphoreId BinSemHandle;
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
void Start_Task01(void const * argument);
void Start_Task02(void const * argument);
void Start_Task03(void const * argument);
void Start_Task04(void const * argument);

/* USER CODE BEGIN PFP */
volatile int flash_wifi = 0; 
volatile int automation_flash = 0; //auto off  

long timeout_to_run_task1 = 0;
long timeout_to_run_task2 = 0;
long timeout_to_run_task3 = 0;
long timeout_to_run_task4 = 0;

char _limit_temp[2] = "30";
int limit_temp = 30;

struct Data_read{
		unsigned char dht11[4];
		unsigned char dht11_old[4];
		float gas;
		float gas_old;
		uint8_t pir;
		uint8_t pir_old;
		uint8_t rain;
		uint8_t rain_old;
		uint8_t light_sensor;
		uint8_t light_sensor_old;
		uint8_t current;
		uint8_t current_old;
		int temperature;
}data_read;

struct Data_send{
		char DHT11[18];
		char GAS[12];
	
		char LED1[6];
		char LED2[6];
		char LED3[6];
		char LED4[6];

		char RELAY[7];
		char FAN1[6];
		char FAN2[6];
	
		char ERASE[7]; //to erase flash EEPROM
		char CURRENT[9]; //current sensor
		
		char GATE[6]; //servo cua chinh
		char POLE[6]; //servo xao phoi do
		char WINDOW[8]; //servo cua so
}data_send;


struct Firebase{
		int FAN1_state;
		int FAN2_state;
		int RELAY_state;
		int LED1_state;
		int LED2_state;
		int LED3_state;
		int LED4_state;
		int GATE_state;
		int POLE_state;
		int WINDOW_state;
}FB;

struct Device{
		int FAN1_state;
		int FAN2_state;
		int RELAY_state;
		int LED1_state;
		int LED2_state;
		int LED3_state;
		int LED4_state;
		int GATE_state;
		int	POLE_state;
		int WINDOW_state;
			
		int FAN1_old_state;
		int FAN2_old_state;
		int RELAY_old_state;
		int LED1_old_state;
		int LED2_old_state;
		int LED3_old_state;
		int LED4_old_state;
		int GATE_old_state;
		int POLE_old_state;
		int WINDOW_old_state;
}DV;

struct Touch{
		int state1;
		int state2;
		int state3;
		int counter_1;
		int counter_2;
}touch;

struct Display{
		char DHT[16];
		char GAS[16];
}display;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Code for comunication with esp8266 using UART 1 and UART 6 */
/*	define */
#define txBuf_size 30
#define rxBuf_size 50

char tx_Buf[txBuf_size];
char rx_Buf[rxBuf_size];

char rxBuf_receive[2];
uint16_t rx_Index = 0;
char temp_receive[rxBuf_size]; //data temp read from esp

/* Function to comunication */
void serial_send_cmd(char data[]){
		if (flash_wifi == 1){	
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
			char line[txBuf_size];
			snprintf(line, sizeof(line), "%s\r\n", data);
			HAL_UART_Transmit(&huart1, (uint8_t*)line, strlen(line), 100);
		}
}

char* serial_read(){
		strcpy(temp_receive, rx_Buf);
		return temp_receive;
}

/* Function to read sensor */
void readSensor(){
		read_DHT11(data_read.dht11);
		data_read.gas = read_gas_ppm();
		data_read.current = read_Current(70);
		if ((data_read.dht11_old[0] != data_read.dht11[0])||(data_read.dht11_old[2] != data_read.dht11[2])||(data_read.dht11_old[1] != data_read.dht11[1])||(data_read.dht11_old[3] != data_read.dht11[3])){
			memcpy(data_read.dht11_old, data_read.dht11, 4);
			//meaning data change 
			data_read.temperature = data_read.dht11_old[2];
			sprintf(data_send.DHT11, "DHT11 %d.%d %d.%d" ,data_read.dht11_old[2],data_read.dht11_old[3],data_read.dht11_old[0],data_read.dht11_old[1]);
			sprintf(display.DHT, "%d.%d\xDF\x43  %d.%d\x37",data_read.dht11_old[2],data_read.dht11_old[3],data_read.dht11_old[0],data_read.dht11_old[1]);
			serial_send_cmd(data_send.DHT11);
			HAL_Delay(30);
		}
		
		if (data_read.gas_old != data_read.gas){
			data_read.gas_old = data_read.gas;
			sprintf(data_send.GAS, "GAS %4.2f" ,data_read.gas_old);
			sprintf(display.GAS, "GAS %4.2f ppm", data_read.gas_old);
			serial_send_cmd(data_send.GAS);
			HAL_Delay(30);
			if (data_read.gas_old >= GAS_thresh){
				control_Fan(2, 1);
				music_play(500);
				HAL_Delay(200);
				music_play(700);
				HAL_Delay(200);
			}	
			else{
				control_Fan(2, 0);
				music_stop();
			}
		}	
		
		if (data_read.current_old != data_read.current){
			data_read.current_old = data_read.current;
			sprintf(data_send.CURRENT, "CURRENT %d", data_read.current_old);
			serial_send_cmd(data_send.CURRENT);
			HAL_Delay(30);
		}
		
//		uint16_t c = 0; //do thay doi cua cam bien gas
//		c = (data_read.gas_old > read_GAS()) ? data_read.gas_old - read_GAS(): read_GAS() - data_read.gas_old;
//		if (c > 0){
//			data_read.gas_old = read_GAS();
//			sprintf(data_send.GAS, "GAS %d\r\n" ,data_read.gas_old);
//			sprintf(display.GAS, "GAS %d ppm", data_read.gas_old);
//			serial_send_cmd(data_send.GAS);
//			HAL_Delay(500);
//		}	
}

/* stream Firebase */
void stream(){
	char temp[2];
	//serial_read();
	//check wifi
	flash_wifi = 1;//connected
	if (strncmp(temp_receive,"WIFI",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '2'){
			flash_wifi = 0; //smartconfig
		}
	}
	
	if (strncmp(temp_receive,"LTMP",4)==0){
		_limit_temp[0] = temp_receive[4];
		_limit_temp[1] = temp_receive[5];
		limit_temp = atoi(_limit_temp);
	}
	
	//stream FAN state
	if (strncmp(temp_receive,"FAN1",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.FAN1_state = 0;
		}
		if (temp[0] == '1'){
			FB.FAN1_state = 1;
		}
	}
	if (strncmp(temp_receive,"FAN2",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.FAN2_state = 0;
		}
		if (temp[0] == '1'){
			FB.FAN2_state = 1;
		}
	}
	//stream RELAY state
	if (strncmp(temp_receive,"RELAY",5)==0){
		temp[0] = temp_receive[5];
		if (temp[0] == '0'){
			FB.RELAY_state = 0;
		}
		if (temp[0] == '1'){
			FB.RELAY_state = 1;
		}
	}
	//stream LED1 state
	if (strncmp(temp_receive,"LED1",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED1_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED1_state = 1;
		}
	}
	//stream LED2 state 
	if (strncmp(temp_receive,"LED2",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED2_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED2_state = 1;
		}
	}	
	//stream lED3 state
	if (strncmp(temp_receive,"LED3",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED3_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED3_state = 1;
		}
	}	
	//stream LED4 state
	if (strncmp(temp_receive,"LED4",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.LED4_state = 0;
		}
		if (temp[0] == '1'){
			FB.LED4_state = 1;
		}
	}	
	//stream GATE state
	if (strncmp(temp_receive,"GATE",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.GATE_state = 0;
		}
		if (temp[0] == '1'){
			FB.GATE_state = 1;
		}
	}	
	
	//stream POLE state
	if (strncmp(temp_receive,"POLE",4)==0){
		temp[0] = temp_receive[4];
		if (temp[0] == '0'){
			FB.POLE_state = 0;
		}
		if (temp[0] == '1'){
			FB.POLE_state = 1;
		}
	}

	//stream Window state
	if (strncmp(temp_receive,"WINDOW",6)==0){
		temp[0] = temp_receive[6];
		if (temp[0] == '0'){
			FB.WINDOW_state = 0;
		}
		if (temp[0] == '1'){
			FB.WINDOW_state = 1;
		}
	}		
}

void save_stream(){
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
		if (DV.GATE_state != FB.GATE_state){
			DV.GATE_state = FB.GATE_state;
		}	
		if (DV.POLE_state != FB.POLE_state){
			DV.POLE_state = FB.POLE_state;
		}	
		if (DV.WINDOW_state != FB.WINDOW_state){
			DV.WINDOW_state = FB.WINDOW_state;
		}	
}

void stream_handle(){
	if (flash_wifi == 1){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	}
	if (flash_wifi == 0){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		osDelay(100);
	}
	if (DV.LED1_state != DV.LED1_old_state){
		DV.LED1_old_state = DV.LED1_state;
		control_Led(1, DV.LED1_old_state);
		sprintf(data_send.LED1, "LED1 %d" ,DV.LED1_old_state);
		serial_send_cmd(data_send.LED1);
		HAL_Delay(30);
	}
	if (DV.LED2_state != DV.LED2_old_state){
		DV.LED2_old_state = DV.LED2_state;
		control_Led(2, DV.LED2_old_state);
		sprintf(data_send.LED2, "LED2 %d" ,DV.LED2_old_state);
		serial_send_cmd(data_send.LED2);
		HAL_Delay(30);
	}
	
	if (DV.LED3_state != DV.LED3_old_state){
		DV.LED3_old_state = DV.LED3_state;
		control_Led(3, DV.LED3_old_state);
		sprintf(data_send.LED3, "LED3 %d" ,DV.LED3_old_state);
		serial_send_cmd(data_send.LED3);
		HAL_Delay(30);
	}
	if (DV.LED4_state != DV.LED4_old_state){
		DV.LED4_old_state = DV.LED4_state;
		control_Led(4, DV.LED4_old_state);
		sprintf(data_send.LED4, "LED4 %d" ,DV.LED4_old_state);
		serial_send_cmd(data_send.LED4);
		HAL_Delay(30);
	}
	if (DV.RELAY_state != DV.RELAY_old_state){
		DV.RELAY_old_state = DV.RELAY_state;
		control_Relay(DV.RELAY_old_state);
		sprintf(data_send.RELAY, "RELAY %d" ,DV.RELAY_old_state);
		serial_send_cmd(data_send.RELAY);
		HAL_Delay(30);
	}
	if (DV.FAN1_state != DV.FAN1_old_state){
		DV.FAN1_old_state = DV.FAN1_state;
		control_Fan(1, DV.FAN1_old_state);
		sprintf(data_send.FAN1, "FAN1 %d" ,DV.FAN1_old_state);
		serial_send_cmd(data_send.FAN1);
		HAL_Delay(30);
	}
	if (DV.FAN2_state != DV.FAN2_old_state){
		DV.FAN2_old_state = DV.FAN2_state;
		control_Fan(2, DV.FAN2_old_state);
		sprintf(data_send.FAN2, "FAN2 %d" ,DV.FAN2_old_state);
		serial_send_cmd(data_send.FAN2);
		HAL_Delay(30);
	}
	if (DV.GATE_state != DV.GATE_old_state){
		DV.GATE_old_state = DV.GATE_state;
		control_Gate(DV.GATE_old_state);
		sprintf(data_send.GATE, "GATE %d" ,DV.GATE_old_state);
		serial_send_cmd(data_send.GATE);
		HAL_Delay(30);
	}
	if (DV.POLE_state != DV.POLE_old_state){
		DV.POLE_old_state = DV.POLE_state;
		control_Pole(DV.POLE_old_state);
		sprintf(data_send.POLE, "POLE %d" ,DV.POLE_old_state);
		serial_send_cmd(data_send.POLE);
		HAL_Delay(30);
	}
	if (DV.WINDOW_state != DV.WINDOW_old_state){
		DV.WINDOW_old_state = DV.WINDOW_state;
		control_Window(DV.WINDOW_old_state);
		sprintf(data_send.WINDOW, "WINDOW %d" ,DV.WINDOW_old_state);
		serial_send_cmd(data_send.WINDOW);
		HAL_Delay(30);
	}
}

/* end of stream */

void lcd_display(){
		LCD_16x2_Clear();
		LCD_16x2_SetCursor(1,1);
		LCD_16x2_Send_String(display.DHT, STR_NOSLIDE);
		LCD_16x2_SetCursor(2,1);
		LCD_16x2_Send_String(display.GAS, STR_NOSLIDE);
}

void smart_control(){
	/* Cau hinh 1 nut bat tat che do tu dong */
	if (automation_flash == 1){
		data_read.pir = read_PIR();
		data_read.light_sensor = read_Light_sensor();
		data_read.rain = read_Rain();
		
		/* Doc cam bien PIR dieu khien den led phong khach tu dong */
		control_Led(1, !data_read.pir);
		FB.LED1_state = !data_read.pir; //auto update state of led1 to firebase

		/* Doc cam bien anh sang dieu khien led san vuon tu dong */
		control_Led(4, !data_read.light_sensor);
		FB.LED4_state = !data_read.light_sensor;
		
		/* Thu sao phoi do tu dong */
		if (data_read.rain == 1){
				servo_position(2, 70);
				HAL_Delay(100);
		}
		else{
				servo_position(2, 0);
				HAL_Delay(100);
		}
	
		/* Nhiet do cao tu dong bat quat phong khach */
		if (data_read.temperature > limit_temp){
				control_Fan(1, 1);
				FB.FAN1_state = 1;
		}
		else{
				control_Fan(1, 0);
				FB.FAN1_state = 0;
		}
	
	/* Nhiet do cao tu bat quat phong khach */
//	if (data_read.temperature > limit_temp){
//			DV.FAN1_state = 1;
//	}
//	else{
//			DV.FAN1_state = 0;
//	}
	
	/* Phat hien khi gas bat quat nha bep */
	}
}

/* Code for comunication with R305 using UART 3*/
uint8_t txBuf[32];
uint8_t rxBuf[32];
uint8_t rxData;
uint8_t rxIndex = 0;

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
			strcpy(temp_receive, rx_Buf);
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12); //bao hieu co data gui tu esp toi stm32
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
	
	touch.state1 = 0;
	touch.counter_1 = 0;
	touch.counter_2 = 0;
	
	LCD_16x2_i2cDeviceCheck();
	LCD_16x2_Init();
	LCD_16x2_BackLight(LCD_BL_ON);
	LCD_16x2_SetCursor(1,1);
	LCD_16x2_Send_String("GOOD DAY MASTER",STR_NOSLIDE);

	LCD_20x4_i2cDeviceCheck();
	LCD_20x4_Init();
	LCD_20x4_BackLight(LCD_BL_ON);
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
	LCD_20x4_SetCursor(2,1);
	LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
	LCD_20x4_SetCursor(3,1);
	LCD_20x4_Send_String("C: Setting",STR_NOSLIDE);
	
	/* Begin receive data from uart 3 and uart 6*/
	HAL_UART_Receive_IT(&huart6 , (uint8_t*)rxBuf_receive, 1);
	//HAL_UART_Receive_IT(&huart3, &rxData, 1);
	HAL_UART_Receive_IT(&huart3, &rxData, 1);
	
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinSem */
  osSemaphoreDef(BinSem);
  BinSemHandle = osSemaphoreCreate(osSemaphore(BinSem), 1);

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
  /* definition and creation of Task01 */
  osThreadDef(Task01, Start_Task01, osPriorityHigh, 0, 128);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, Start_Task02, osPriorityAboveNormal, 0, 128);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, Start_Task03, osPriorityNormal, 0, 128);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* definition and creation of Task04 */
  osThreadDef(Task04, Start_Task04, osPriorityLow, 0, 128);
  Task04Handle = osThreadCreate(osThread(Task04), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 900-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /*Configure GPIO pins : PA1 PA3 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
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

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*Button 1*/
	if (GPIO_Pin == GPIO_PIN_15){
		if (touch.counter_1 > 0)
		{
			touch.state1 = !touch.state1;
			control_Relay(touch.state1);
		}
		touch.counter_1 ++;
		if (touch.counter_1 == 2){
			touch.counter_1 = 1;
		}		
	}
	
	/*Button 2*/
	if (GPIO_Pin == GPIO_PIN_13){
		if (touch.counter_2 > 0)
		{
			automation_flash = !automation_flash;
			if (automation_flash == 1){
				LCD_16x2_Clear();
				LCD_16x2_SetCursor(1,1);
				LCD_16x2_Send_String("Auto mode is on", STR_NOSLIDE);
				LCD_16x2_SetCursor(2,1);
				LCD_16x2_Send_String("Limit temp:", STR_NOSLIDE);
				LCD_16x2_SetCursor(2, 15);
				LCD_16x2_Send_String(_limit_temp, STR_NOSLIDE);
				HAL_Delay(200);
			}
			else{
				LCD_16x2_Clear();
				LCD_16x2_SetCursor(1,1);
				LCD_16x2_Send_String("Auto mode is off", STR_NOSLIDE);
				HAL_Delay(200);
			}
		}
		touch.counter_2 ++;
		if (touch.counter_2 == 2){
			touch.counter_2 = 1;
		}	
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Task01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Task01 */
void Start_Task01(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		stream();
		save_stream();
		stream_handle();
		osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Task02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task02 */
void Start_Task02(void const * argument)
{
  /* USER CODE BEGIN Start_Task02 */
	/* Task dung cho doc cam bien: gas, dht11, current, hien thi lcd*/
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GetTick() - timeout_to_run_task2 >= 7500){
			readSensor();
		//osSemaphoreWait(BinSemHandle, osWaitForever);
			lcd_display();
		//osSemaphoreRelease(BinSemHandle);
			timeout_to_run_task2 = HAL_GetTick();
		}
    osDelay(10);
  }
  /* USER CODE END Start_Task02 */
}

/* USER CODE BEGIN Header_Start_Task03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task03 */
void Start_Task03(void const * argument)
{
  /* USER CODE BEGIN Start_Task03 */
	//HAL_UART_Receive_IT(&huart3, &rxData, 1);
  /* Infinite loop */
  for(;;)
  {
		//osSemaphoreWait(BinSemHandle, osWaitForever);
		verify_password();
		//osSemaphoreRelease(BinSemHandle);
//		if(HAL_GetTick() - timeout_to_run_task3 >= 5000){
//			verify_password();
//			timeout_to_run_task3 = HAL_GetTick();
//		}
    osDelay(10);
  }
  /* USER CODE END Start_Task03 */
}

/* USER CODE BEGIN Header_Start_Task04 */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task04 */
void Start_Task04(void const * argument)
{
  /* USER CODE BEGIN Start_Task04 */
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GetTick() - timeout_to_run_task4 >= 10000){
			smart_control();
			timeout_to_run_task4 = HAL_GetTick();
		}
    osDelay(10);
  }
  /* USER CODE END Start_Task04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
