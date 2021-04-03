#include "lib_keypad.h"

uint8_t key;
uint8_t STR[4] = {'1', '2', '3', '4'};
uint8_t str[4] = {' ', ' ', ' ', ' '};
uint8_t kitu = 0;
uint8_t countkitu =0;
uint8_t kiemtra = 0;
#define C1_PORT GPIOD
#define C1_PIN GPIO_PIN_14

#define C2_PORT GPIOD
#define C2_PIN GPIO_PIN_13

#define C3_PORT GPIOD
#define C3_PIN GPIO_PIN_12

#define C4_PORT GPIOD
#define C4_PIN GPIO_PIN_11

#define R1_PORT GPIOD
#define R1_PIN GPIO_PIN_10

#define R2_PORT GPIOD
#define R2_PIN GPIO_PIN_9

#define R3_PORT GPIOD
#define R3_PIN GPIO_PIN_8

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_15

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








//____________________________________________________

void dangnhap()
{
		key = read_keypad ();
		if(key != ' ')
  {
    if(kitu ==0){
      str[0] = key;
      lcd_send_cmd (0x80|0x47);
      lcd_send_data(str[0]);
      HAL_Delay(1000); // Ký t? hi?n th? trên màn hình LCD trong 1
      lcd_send_cmd (0x80|0x47);
      lcd_send_string("*"); // Ký t? du?c che b?i d?u 
      }
    if(kitu == 1){
      str[1] = key;
      lcd_send_cmd (0x80|0x48);
      lcd_send_data(str[1]);
      HAL_Delay(1000); // Ký t? hi?n th? trên màn hình LCD trong 1
      lcd_send_cmd (0x80|0x48);
      lcd_send_string("*");// Ký t? du?c che b?i d?u
      }
    if(kitu == 2) {
      str[2] = key;
      lcd_send_cmd (0x80|0x49);
      lcd_send_data(str[2]);
      HAL_Delay(1000); // Ký t? hi?n th? trên màn hình LCD trong 1
      lcd_send_cmd (0x80|0x49);
      lcd_send_string("*"); // Ký t? du?c che b?i d?u
      }
    if(kitu == 3) {
      str[3] = key;
      lcd_send_cmd (0x80|0x4A);
      lcd_send_data(str[3]);
      HAL_Delay(1000); // Ký t? hi?n th? trên màn hình LCD trong 1
      lcd_send_cmd (0x80|0x4A);
      lcd_send_string("*"); // Ký t? du?c che b?i d?u 
      countkitu = 1;
    }
    kitu = kitu+1;
  }
 
  if(countkitu == 1) {
  if(str[0] == STR[0] && str[1] == STR[1] && str[2] == STR[2] && str[3] == STR[3]) 
    {
      lcd_clear();
			lcd_send_cmd (0x80|0x00);
      lcd_send_string("    Correct!");
      HAL_Delay(1000);
			lcd_clear();
			lcd_send_cmd (0x80|0x00);
      lcd_send_string("    Opened!");
			lcd_send_cmd (0x80|0x40);
			lcd_send_string("A=Enroll");
			lcd_send_cmd (0x80|0x14);
			lcd_send_string("B=Change Password");
			lcd_send_cmd (0x80|0x54);
			lcd_send_string("C=Delete     D=Close");
			HAL_Delay(20);
      kitu = 0;
      countkitu = 0;
			kiemtra = 1;
			while(kiemtra)
			{
				key = read_keypad ();
				if(key == 'D')
					break;
				while(key=='B')
				{
					doimatkhau();
					kiemtra = 0;
					break;
					
				}
			}
			
		
		} 
		else 
		{
      lcd_clear();
			lcd_send_cmd (0x80|0x00);
      lcd_send_string("    Incorrect!");
      HAL_Delay(1000);
      lcd_clear();
			lcd_send_cmd (0x80|0x00);
      lcd_send_string("    Try Again!");
      HAL_Delay(1000);
      lcd_clear();
			lcd_send_cmd (0x80|0x00);
      lcd_send_string(" Enter Password");
			lcd_send_cmd (0x80|0x47);
			lcd_send_string("####");
      kitu = 0;
      countkitu = 0;
		}
		
		}
	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
	switch(key){
		case 'D':
		{
			lcd_clear();
			lcd_send_cmd (0x80|0x00);
			lcd_send_string("       Closed!");
			
			HAL_Delay(1000);
			lcd_clear();
			lcd_send_cmd (0x80|0x00);
			lcd_send_string(" Enter Password");
			lcd_send_cmd (0x80|0x47);
			lcd_send_string("####");
			kitu = 0;
			countkitu = 0;
			kiemtra = 0;
    break;
		}
		}
	}

	//_____________________________________
	void doimatkhau(void)
	{
		
		lcd_clear();
		lcd_send_cmd (0x80|0x00);
		lcd_send_string ("Enter new password: ");
		lcd_send_cmd (0x80|0x47);
			lcd_send_string("####");
		uint8_t newpass[9] = {'5', ' ', ' ', ' ',' ', ' ', ' ', ' ', ' '};
		uint8_t j=1 ,countj=0;
		while(j!=9)
		{ 
						key = read_keypad ();
						
							if(key == 'D')
								break;
							if(key != ' ')
						{
							HAL_Delay(20);
							if(j ==1){
								newpass[1] = key;
								lcd_send_cmd (0x80|0x47);
								lcd_send_data(newpass[1]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x47);
								lcd_send_string("*"); // K? t? du?c che b?i d?u 
								}
							if(j == 2){
								newpass[2] = key;
								lcd_send_cmd (0x80|0x48);
								lcd_send_data(newpass[2]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x48);
								lcd_send_string("*");// K? t? du?c che b?i d?u
								}
							if(j == 3) {
								newpass[3] = key;
								lcd_send_cmd (0x80|0x49);
								lcd_send_data(newpass[3]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x49);
								lcd_send_string("*"); // K? t? du?c che b?i d?u
								}
							if(j == 4) {
								newpass[4] = key;
								lcd_send_cmd (0x80|0x4A);
								lcd_send_data(newpass[4]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x4A);
								lcd_send_string("*"); // K? t? du?c che b?i d?u 
								HAL_Delay(1000);
								lcd_clear();
								lcd_send_cmd (0x80|0x00);
								lcd_send_string ("Confirm new password");
								lcd_send_cmd (0x80|0x47);
								lcd_send_string("####");
								}
							if(j ==5){
								newpass[5] = key;
								lcd_send_cmd (0x80|0x47);
								lcd_send_data(newpass[5]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x47);
								lcd_send_string("*"); // K? t? du?c che b?i d?u 
								}
							if(j == 6){
								newpass[6] = key;
								lcd_send_cmd (0x80|0x48);
								lcd_send_data(newpass[6]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x48);
								lcd_send_string("*");// K? t? du?c che b?i d?u
								}
							if(j == 7) {
								newpass[7] = key;
								lcd_send_cmd (0x80|0x49);
								lcd_send_data(newpass[7]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x49);
								lcd_send_string("*"); // K? t? du?c che b?i d?u
								}
							if(j == 8) {
								newpass[8] = key;
								lcd_send_cmd (0x80|0x4A);
								lcd_send_data(newpass[8]);
								HAL_Delay(1000); // K? t? hi?n th? tr?n m?n h?nh LCD trong 1
								lcd_send_cmd (0x80|0x4A);
								lcd_send_string("*");								// K? t? du?c che b?i d?u 
								countj=1;
								}
							j = j+1;
						}
						if(countj==1){
						if(newpass[1] == newpass[5] && newpass[2] == newpass[6] && newpass[3] == newpass[7] && newpass[4] == newpass[8])
						{
							lcd_clear();
							lcd_send_cmd (0x80|0x00);
							lcd_send_string ("Done! ");
							STR[0] = newpass[1];
              STR[1] = newpass[2];
              STR[2] = newpass[3];
              STR[3] = newpass[4];
							HAL_Delay(2000);
							lcd_send_cmd (0x80|0x00);
							lcd_send_string ("Enter Password ");
							lcd_send_cmd (0x80|0x47);
							lcd_send_string("####");
							
						}
						
						else{
							j=1;
							countj=0;
							lcd_clear();
							lcd_send_cmd (0x80|0x00);
							lcd_send_string ("Didn't match! ");
							HAL_Delay(2000);
							lcd_send_cmd (0x80|0x00);
							lcd_send_string ("Try Again!      ");
							HAL_Delay(2000);
							lcd_send_cmd (0x80|0x00);
							lcd_send_string ("Enter new password: ");
							lcd_send_cmd (0x80|0x47);
							lcd_send_string("####");
						}
					}
		}
	}


