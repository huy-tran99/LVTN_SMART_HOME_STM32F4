#include "lib_keypad.h"
#include "finger.h"
#include "main.h"

extern uint8_t PageID;
extern uint8_t test;

extern uint8_t key;
uint8_t STR[4] = {'1', '2', '3', '4'};
uint8_t str[4] = {' ', ' ', ' ', ' '};
uint8_t kitu = 0;
uint8_t countkitu =0;
uint8_t kiemtra = 0;
uint8_t block = 0;

#define timeout 5000 //ms 

#define C1_PORT GPIOB
#define C1_PIN GPIO_PIN_3

#define C2_PORT GPIOD
#define C2_PIN GPIO_PIN_6

#define C3_PORT GPIOD
#define C3_PIN GPIO_PIN_4

#define C4_PORT GPIOD
#define C4_PIN GPIO_PIN_2

#define R1_PORT GPIOD
#define R1_PIN GPIO_PIN_0

#define R2_PORT GPIOC
#define R2_PIN GPIO_PIN_11

#define R3_PORT GPIOC
#define R3_PIN GPIO_PIN_15

#define R4_PORT GPIOA
#define R4_PIN GPIO_PIN_7

char read_keypad (void)
{
		/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '1';
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '2';
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '3';
	}
	
	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'A';
	}
	HAL_Delay(10);
	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '4';
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '5';
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '6';
	}
	
	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'B';
	}
	HAL_Delay(10);
	
	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '7';
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '8';
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '9';
	}
	
	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'C';
	}
	HAL_Delay(10);
		
	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));   // wait till the button is pressed
		return '*';
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));   // wait till the button is pressed
		return '0';
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));   // wait till the button is pressed
		return '#';
	}
	
	if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'D';
	}
	
	HAL_Delay(10);
	return ' ';
}

//#########################################################################################################################
void Enter()
{
	key = read_keypad ();

	if (key == 'A')
	{
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String("Hold D: Back. ",STR_NOSLIDE);
		HAL_Delay(500);
		while(1)
		{
			test = FingerPrintFast();
			key = read_keypad ();
			if (test == 1)
			{	
				MOCUA;
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("             D=Close",STR_NOSLIDE);
				
				uint64_t timestart = HAL_GetTick();
				while(1)
				{
					uint64_t timefinish = HAL_GetTick();
					key = read_keypad ();
					if(key == 'D'||timefinish-timestart >= timeout)
					{
						LCD_20x4_Clear(); 
						LCD_20x4_SetCursor(1,1);
						LCD_20x4_Send_String("       Closed!",STR_NOSLIDE);
						DONGCUA;
						HAL_Delay(1000);
						LCD_20x4_Clear();
						LCD_20x4_SetCursor(1,1);
						LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
						LCD_20x4_SetCursor(2,1);
						LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
						key = 'D';
						break;
					}
						
				}
				HAL_Delay(500);
				break;
			}
			if(key == 'D')
			{
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
				LCD_20x4_SetCursor(2,1);
				LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
				HAL_Delay(500);
				break;
			}
		}
	}

	if (key == 'B')
	{
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String(" Enter Password",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,4);
		LCD_20x4_Send_String("####",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String("D: Back. ",STR_NOSLIDE);	
		HAL_Delay(100);
		while(1)
		{
			EnterByPassword();
			if(key == 'D')
			{
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
				LCD_20x4_SetCursor(2,1);
				LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
				break;
			}				
		}
	}
	HAL_Delay(100);
}

//#########################################################################################################################

void EnterByPassword()
{
	key = read_keypad ();
	if(key != ' ')
  	{
		if(kitu == 0)
		{
			str[0] = key;
			LCD_20x4_SetCursor(2,4);
			LCD_20x4_Write_Data(str[0]);
			HAL_Delay(500); 
			LCD_20x4_SetCursor(2,4);
			LCD_20x4_Send_String("*",STR_NOSLIDE);  
		}
		if(kitu == 1)
		{
			str[1] = key;
			LCD_20x4_SetCursor(2,5);
			LCD_20x4_Write_Data(str[1]);
			HAL_Delay(500); 
			LCD_20x4_SetCursor(2,5);
			LCD_20x4_Send_String("*",STR_NOSLIDE);
		}
		if(kitu == 2) 
		{
			str[2] = key;
			LCD_20x4_SetCursor(2,6);
			LCD_20x4_Write_Data(str[2]);
			HAL_Delay(500); 
			LCD_20x4_SetCursor(2,6);
			LCD_20x4_Send_String("*",STR_NOSLIDE); 
		}
		if(kitu == 3) 
		{
			str[3] = key;
			LCD_20x4_SetCursor(2,7);
			LCD_20x4_Write_Data(str[3]);
			HAL_Delay(500); 
			LCD_20x4_SetCursor(2,7);
			LCD_20x4_Send_String("*",STR_NOSLIDE);
			countkitu = 1;
		}
		kitu = kitu + 1;
  	}
 
  	if(countkitu == 1) 
	{
 		if(str[0] == STR[0] && str[1] == STR[1] && str[2] == STR[2] && str[3] == STR[3]) 
    	{
			block = 0;
      		LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
      		LCD_20x4_Send_String("    Correct!",STR_NOSLIDE);
      		HAL_Delay(1000);    
			MOCUA;
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
      		LCD_20x4_Send_String("    Opened!",STR_NOSLIDE);
			LCD_20x4_SetCursor(2,1);
			LCD_20x4_Send_String("A=Enroll",STR_NOSLIDE);
			LCD_20x4_SetCursor(3,1);
			LCD_20x4_Send_String("B=Change Password",STR_NOSLIDE);
			LCD_20x4_SetCursor(4,1);
			LCD_20x4_Send_String("C=Delete     D=Close",STR_NOSLIDE);
			HAL_Delay(20);
      		kitu = 0;
      		countkitu = 0;
			kiemtra = 1;
			uint64_t timestart = HAL_GetTick();
			while(kiemtra)
			{
				uint64_t timefinish = HAL_GetTick();
				key = read_keypad ();
				if(key == 'D'||timefinish-timestart >= timeout)
				{
					key = 'D';
					break;
				}
				while(key == 'A')
				{
					HAL_Delay(300);
					LCD_20x4_Clear();
					LCD_20x4_SetCursor(1,1);
      				LCD_20x4_Send_String("Enroll ID: ",STR_NOSLIDE);
					LCD_20x4_SetCursor(2,1);
      				LCD_20x4_Send_String("###   (1-234)",STR_NOSLIDE);
      				HAL_Delay(100);
					PageID = 0;
					PageID = readnumber();
					if (PageID == 0) // ID #0 not allowed, try again!
               			return;
					while(! Enroll() );/* while(! Enroll() ); Nhớ chèn hàm if (key == 'D') vô hàm con Enroll */
						kiemtra = 0;
					break;
				}
				
				while(key=='B')
				{
					HAL_Delay(100);
					ChangePassword();
					kiemtra = 0;
					break;					
				}

				while (key == 'C')
				{
					HAL_Delay(300); 
					LCD_20x4_Clear();
					LCD_20x4_SetCursor(1,1);
      				LCD_20x4_Send_String("Delete ID: ",STR_NOSLIDE);
					LCD_20x4_SetCursor(2,1);
      				LCD_20x4_Send_String("###   (1-234)",STR_NOSLIDE);
					LCD_20x4_SetCursor(4,1);
					LCD_20x4_Send_String("             D=Close",STR_NOSLIDE);
      				HAL_Delay(100);
					PageID = 0;
					PageID = readnumber();
					if (PageID == 0) // ID #0 not allowed, try again!
               			return;
					while ( deleteModel(PageID));  /* while( deleteModel(PageID)); Nho them if (key == 'D') trong ham con deleteModel */
					
					LCD_20x4_SetCursor(1,1);
      				LCD_20x4_Send_String("Deleted ID: ",STR_NOSLIDE);
					HAL_Delay(100);
					kiemtra = 0;
					break;
				}
			}		
		} 
		else 
		{
			block++;
      		LCD_20x4_Clear(); 
			LCD_20x4_SetCursor(1,1);
      		LCD_20x4_Send_String("    Incorrect!",STR_NOSLIDE);
      		HAL_Delay(1000);
      		LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
      		LCD_20x4_Send_String("    Try Again!",STR_NOSLIDE);
      		HAL_Delay(1000);
      		LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
      		LCD_20x4_Send_String("Enter Password",STR_NOSLIDE);
			LCD_20x4_SetCursor(2,4);
			LCD_20x4_Send_String("####",STR_NOSLIDE);
			LCD_20x4_SetCursor(4,1);
			LCD_20x4_Send_String("D: Back",STR_NOSLIDE);
      		kitu = 0;
      		countkitu = 0;
		}		
	}
	
	if (block == 4)
	{
		BlockFunction();
	}
			
	switch(key)
	{
		case 'D':
		{
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("       Closed!",STR_NOSLIDE);
			DONGCUA;
			HAL_Delay(1000);
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(4,1);
			LCD_20x4_Send_String("D: Back ",STR_NOSLIDE);
			kitu = 0;
			countkitu = 0;
			kiemtra = 0;
    		break;
		}
	}
}

void ChangePassword(void)
{	
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String ("Enter new password: ",STR_NOSLIDE);
	LCD_20x4_SetCursor(2,4);
	LCD_20x4_Send_String("####",STR_NOSLIDE);
	uint8_t newpass[9] = {'5', ' ', ' ', ' ',' ', ' ', ' ', ' ', ' '};
	uint8_t j=1 ,countj=0;
	while(j != 9)
	{ 
		key = read_keypad ();					
		if (key == 'D')
			break;
		if (key != ' ')
		{
			HAL_Delay(20);
			if(j ==1){
				newpass[1] = key;
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Write_Data(newpass[1]);
				HAL_Delay(500); 
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Send_String("*",STR_NOSLIDE); 
			}
			if(j == 2){
				newpass[2] = key;
				LCD_20x4_SetCursor(2,5);
				LCD_20x4_Write_Data(newpass[2]);
				HAL_Delay(500); 
				LCD_20x4_SetCursor(2,5);
				LCD_20x4_Send_String("*",STR_NOSLIDE);
			}
			if(j == 3) {
				newpass[3] = key;
				LCD_20x4_SetCursor(2,6);
				LCD_20x4_Write_Data(newpass[3]);
				HAL_Delay(500); 
				LCD_20x4_SetCursor(2,6);
				LCD_20x4_Send_String("*",STR_NOSLIDE); 
			}
			if(j == 4) {
				newpass[4] = key;
				LCD_20x4_SetCursor(2,7);
				LCD_20x4_Write_Data(newpass[4]);
				HAL_Delay(500);
				LCD_20x4_SetCursor(2,7);
				LCD_20x4_Send_String("*",STR_NOSLIDE);  
				HAL_Delay(1000);
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Confirm new password",STR_NOSLIDE);
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Send_String("####",STR_NOSLIDE);
			}
			if(j ==5){
				newpass[5] = key;
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Write_Data(newpass[5]);
				HAL_Delay(500); 
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Send_String("*",STR_NOSLIDE); 
			}
			if(j == 6){
				newpass[6] = key;
				LCD_20x4_SetCursor(2,5);
				LCD_20x4_Write_Data(newpass[6]);
				HAL_Delay(500); 
				LCD_20x4_SetCursor(2,5);
				LCD_20x4_Send_String("*",STR_NOSLIDE);
			}
			if(j == 7) {
				newpass[7] = key;
				LCD_20x4_SetCursor(2,6);
				LCD_20x4_Write_Data(newpass[7]);
				HAL_Delay(500);
				LCD_20x4_SetCursor(2,6);
				LCD_20x4_Send_String("*",STR_NOSLIDE); 
			}
			if(j == 8) {
				newpass[8] = key;
				LCD_20x4_SetCursor(2,7);
				LCD_20x4_Write_Data(newpass[8]);
				HAL_Delay(500); 
				LCD_20x4_SetCursor(2,7);
				LCD_20x4_Send_String("*",STR_NOSLIDE);				
				countj = 1;
			}
			j = j + 1;
		}
		if (countj == 1)
		{
			if(newpass[1] == newpass[5] && newpass[2] == newpass[6] && newpass[3] == newpass[7] && newpass[4] == newpass[8])
			{
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Done! ",STR_NOSLIDE);
				STR[0] = newpass[1];
				STR[1] = newpass[2];
				STR[2] = newpass[3];
				STR[3] = newpass[4];
				HAL_Delay(2000);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Enter Password ",STR_NOSLIDE);
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Send_String("####",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("             D=Close",STR_NOSLIDE);
				
			}					
			else{
				j=1;
				countj=0;
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Didn't match! ",STR_NOSLIDE);
				HAL_Delay(2000);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Try Again!      ",STR_NOSLIDE);
				HAL_Delay(2000);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Enter new password: ",STR_NOSLIDE);
				LCD_20x4_SetCursor(2,4);
				LCD_20x4_Send_String("####",STR_NOSLIDE);
			}
		}
	}
}
  
//#########################################################################################################################

uint8_t readnumber(void)
{
	uint16_t num = 0, count = 0, i = 0;
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel and close!",STR_NOSLIDE);
	while (num == 0 || count == 0) 
	{
		key = read_keypad();
		HAL_Delay(10);
		if (key == 'D')
			break;
		if(key != ' ' && key != 'A' && key != 'B' && key != 'C' && key != '*' && key != '#')
		{ 
			HAL_Delay(10);
			if(i ==0)
			{
				num = num + (key-48)*100;
				LCD_20x4_SetCursor(2,1);
				LCD_20x4_Write_Data(key);
				HAL_Delay(500); // Ký tự hiển thị trên màn hình LCD trong 1
			}
			if(i == 1)
			{
				num = num + (key-48)*10;
				LCD_20x4_SetCursor(2,2);
				LCD_20x4_Write_Data(key);
				HAL_Delay(500); // Ký tự được che bởi dấu
			}
			if(i == 2)
			{
				num = num + (key-48);
				count = 1;
				LCD_20x4_SetCursor(2,3);
				LCD_20x4_Write_Data(key);
				if (num >=235)
				{
					key = 'D';
					DONGCUA;
					break;
				}
				HAL_Delay(500); // Ký tự được che bởi dấu
			}
			i = i +1;
		}
    }
  	return num;
}

//#########################################################################################################################
void BlockFunction(void)
{
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
    LCD_20x4_Send_String("Blocked 1 minute! ",STR_NOSLIDE);
	for(int m = 0; m < 100; m++)
	{
		Toggle;
		HAL_Delay(50);
	}
	block = 0;
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
    LCD_20x4_Send_String("Enter Password",STR_NOSLIDE);
	LCD_20x4_SetCursor(2,4);
	LCD_20x4_Send_String("####",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Back",STR_NOSLIDE);
}
