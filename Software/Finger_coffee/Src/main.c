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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "finger.h"
#include "lcd_20x4.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Khai bao buffer */
uint8_t txBuf[32];
uint8_t rxBuf[32];
uint8_t rxData;
uint8_t rxIndex = 0;

/* Khai bao cac bien can thiet*/
int p = -1; //int test = 0;
/* Khai bao ngat uart de nhan data*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		rxBuf[rxIndex] = rxData;	
		if (rxIndex < (sizeof(rxBuf)-2))
			rxIndex++;
		HAL_UART_Receive_IT(&huart2, &rxData, 1);
}

int getFingerprintEnroll(int id){
		p = 0; //test = 0;
		while (p != FINGERPRINT_NOFINGER){
			p = getImage();
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("...",STR_NOSLIDE);
			//test = 1; //...
			HAL_Delay(500);
		}
		while (p != FINGERPRINT_OK){
			p = getImage();
			HAL_Delay(500);
		}
		
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("Image taken",STR_NOSLIDE);
		//test = 2; //Image taken
		HAL_Delay(500);
		p = image2Tz(1);
    HAL_Delay(500);
		if (p == FINGERPRINT_OK){
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String("Image converted",STR_NOSLIDE);
				//test = 3; //Image converted
		}
		else{
			//test = 4;//EROR to converted finger 1
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("ERROR",STR_NOSLIDE);
			return -1;
		}
		HAL_Delay(500);
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("Wait to lift",STR_NOSLIDE);
		//test = 5; //Wait to lift your hand
		p = 0;
		HAL_Delay(2000);
		
		while (p != FINGERPRINT_NOFINGER){
			p = getImage();
			HAL_Delay(500);
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("...",STR_NOSLIDE);
			//test = 6; //...
		}
		//test = 7; //Now get image again
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("Now again",STR_NOSLIDE);
		HAL_Delay(2000);
		
		while (p != FINGERPRINT_OK){
			p = getImage();
			HAL_Delay(500);
		}
	  
		//test = 8; //Image 2 taken
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("Image 2 taken",STR_NOSLIDE);
		HAL_Delay(500);
    p = image2Tz(2);
    HAL_Delay(500);
    if (p == FINGERPRINT_OK){
				//test = 9;//Image converted
					LCD_20x4_Clear();
					LCD_20x4_SetCursor(1,1);
					LCD_20x4_Send_String("Image converted",STR_NOSLIDE);
		}
    else{
        //test = 10;//Error 2
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String("ERROR1",STR_NOSLIDE);
        return -1;
		}
    // OK converted!
		//test = 11; //Creating model for # id
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Print("Creating model:%.0f", id);
    HAL_Delay(1000);
    p = createModel();
    HAL_Delay(1000);
    if (p == FINGERPRINT_OK){
			//test = 12; //Prints matched!
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("Prints matched",STR_NOSLIDE);
			HAL_Delay(100);
		}			
    else{
      //test = 13; //Error
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("ERROR2",STR_NOSLIDE);
			HAL_Delay(100);
      return -1;
		}
    // Store finger
    HAL_Delay(1000);
    p = storeModel(id);
    HAL_Delay(1000);
		if (p == FINGERPRINT_OK){
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("Stored!",STR_NOSLIDE);	
			//test = 14; //Stored !
		}      
    else{
			//test = 15; //ERROR
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("ERROR3",STR_NOSLIDE);
			HAL_Delay(100);
			return -1;
		}
    return 1;	
}

int search_Finger(){
		p = getImage();
		HAL_Delay(100);
		if (p != FINGERPRINT_OK){
			return 0;
		}
		
		HAL_Delay(100);
		p = image2Tz(1);
		HAL_Delay(100);
		if (p != FINGERPRINT_OK){
        return 0; 
		}
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("OK",STR_NOSLIDE);
		HAL_Delay(500);
		
    p = search(); 
		HAL_Delay(1000);
    if (p == FINGERPRINT_OK){
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Print("Found finger:%.0f", returnFingerID());
				HAL_Delay(500);
		}
    else{
        LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String("Not found",STR_NOSLIDE);
				HAL_Delay(500);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	
	/*I2c LCD 16x2*/
	
	LCD_20x4_i2cDeviceCheck();
	LCD_20x4_Init();
	LCD_20x4_BackLight(LCD_BL_ON);
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String("hello",STR_NOSLIDE);
	
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &rxData, 1);
  /* USER CODE END 2 */
	p = empty();
	HAL_Delay(1000);
	
	if (getFingerprintEnroll(100)!= 1){
			getFingerprintEnroll(100);
	}
	
	if (getFingerprintEnroll(20)!= 1){
			getFingerprintEnroll(20);
	}
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		p = search();
//		HAL_Delay(1000);
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("Begin search",STR_NOSLIDE);
		HAL_Delay(1000);
		search_Finger();
		HAL_Delay(1000);
    /* USER CODE END WHILE */
    /* receive data rx */
		//rxIndex = 0;
//		p = getImage();
//		HAL_Delay(1000);
		
		//rxIndex = 0;
//		p = match();
//		HAL_Delay(1000);
		
		//rxIndex = 0;
//		p = empty();
//		HAL_Delay(1000);

		//rxIndex = 0;
//		p = search();
//		HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
