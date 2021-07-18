#include "password.h"

char key_pad_value;
uint8_t test;

char password_setup[4] = "1234";
char password_input[4];
char password_change[4];
char password_confirm[4];

int dir = 0;
int pass_correct = 0;

void open_door(void)
{
	music_play(800);
	servo_position(1, 70);
	HAL_Delay(500);

	music_play(400);
	HAL_Delay(500);
	music_stop();
}

void close_door(void)
{
	servo_position(1, 0);
	HAL_Delay(500);
}


void change_password(void)
{	
	int j = 0;
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String(" Enter New Password",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel. ",STR_NOSLIDE);	
	while( j < 4){
		char passwordkey_change =  read_keypad();
		if((passwordkey_change != NULL) || (passwordkey_change != 'D')){
			if(passwordkey_change){
				password_change[j] = passwordkey_change;
				LCD_20x4_SetCursor(2,(4+j));
				LCD_20x4_Write_Data(password_change[j]);
				HAL_Delay(300);
				j++;
			}
		}
		else if(passwordkey_change == 'D'){
			break;
		}
	}

	j = 0;
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String(" Confirm Password",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel. ",STR_NOSLIDE);
	while( j < 4){
		char passwordkey_confirm =  read_keypad();
		if((passwordkey_confirm != NULL) || (passwordkey_confirm != 'D')){
			if(passwordkey_confirm){
				password_confirm[j] = passwordkey_confirm;
				LCD_20x4_SetCursor(2,(4+j));
				LCD_20x4_Write_Data(password_confirm[j]);
				HAL_Delay(300);
				j++;
			}
		}
		else if(passwordkey_confirm == 'D'){
			break;
		}
	}

	if(strncmp(password_change ,password_confirm ,4) == 0){
		strcpy(password_setup,password_change);
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String(" CHANGE DONE !! ",STR_NOSLIDE);
		HAL_Delay(1000);
	}
	else{
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String(" CHANGE ERROR !! ",STR_NOSLIDE);
		HAL_Delay(1000);
	}
}

  
/* Function to enroll finger */
/*
int finger_enroll(void)
{	
	int p = -1;
	while (p != FINGERPRINT_OK)
   	{
		key_pad_value = read_keypad();
		if (key_pad_value == 'D')
		   break;

		p = getImage();	
		HAL_Delay(500);

    	switch (p)
      	{
         	case FINGERPRINT_OK:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Image taken1",STR_NOSLIDE);
				HAL_Delay(1000);
				break;
			case FINGERPRINT_NOFINGER:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				HAL_Delay(300);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String (".1",STR_NOSLIDE);
				HAL_Delay(300);
				break;
			case FINGERPRINT_PACKETRECIEVEERR:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Communication error",STR_NOSLIDE);
				break;
			case FINGERPRINT_IMAGEFAIL:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Imaging error",STR_NOSLIDE);
				break;
			default:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
				break;
      }
   	}
   	// OK chup hinh xong, gio chuyen hinh sang character
	p = image2Tz(1);
	HAL_Delay(500);

	switch (p) {
		case FINGERPRINT_OK:
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String ("Character1",STR_NOSLIDE);
			HAL_Delay(1000);
			break;
		case FINGERPRINT_IMAGEMESS:
			return p;
		case FINGERPRINT_PACKETRECIEVEERR:
			return p;
		case FINGERPRINT_FEATUREFAIL:
			return p;
		case FINGERPRINT_INVALIDIMAGE:
			return p;
		default:
			return p;
	}
   
   	//Done

	LCD_20x4_Clear();
	LCD_20x4_SetCursor(3,1);
	LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String ("Remove finger",STR_NOSLIDE);
	HAL_Delay(500);

   	p = -1;
   
   	while (p != FINGERPRINT_NOFINGER)
   	{
		key_pad_value = read_keypad();
		if (key_pad_value == 'D')
			break;
      	p = getImage();
      	HAL_Delay(500);      
   	}

   	p = -1;
   
   	while (p != FINGERPRINT_OK) 
   	{	
		key_pad_value = read_keypad();
		if (key_pad_value == 'D')
			break;
      
      	p = getImage();
		HAL_Delay(500);
      	switch (p) 
      	{
         	case FINGERPRINT_OK:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Image taken2",STR_NOSLIDE);
				HAL_Delay(1000);
				break;
         	case FINGERPRINT_NOFINGER:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				HAL_Delay(300);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String (".2",STR_NOSLIDE);
				HAL_Delay(300);
				break;
			case FINGERPRINT_PACKETRECIEVEERR:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Communication error",STR_NOSLIDE);
				break;
			case FINGERPRINT_IMAGEFAIL:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Imaging error",STR_NOSLIDE);
				break;
			default:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
				break;
      	}
   	}

	p = image2Tz(2);
	HAL_Delay(500);		
   	switch (p) 
   	{
      	case FINGERPRINT_OK:
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String ("Character2",STR_NOSLIDE);
			HAL_Delay(1000);
			break;
		case FINGERPRINT_IMAGEMESS:
			return p;
		case FINGERPRINT_PACKETRECIEVEERR:
			return p;
		case FINGERPRINT_FEATUREFAIL:
			return p;
		case FINGERPRINT_INVALIDIMAGE:
			return p;
		default:
			return p;
   	}

   	//OK converted!
	p = createModel();
	HAL_Delay(500);
	if (p == FINGERPRINT_OK) 
	{
		//Prints matched!
	} 
   	else if (p == FINGERPRINT_PACKETRECIEVEERR) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
   	} 
	else if (p == FINGERPRINT_ENROLLMISMATCH) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Not match",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
	} 
	else {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
	}

	p = storeModel(PageID);
	HAL_Delay(500);	
	if (p == FINGERPRINT_OK) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Stored ",STR_NOSLIDE);
		LCD_20x4_SetCursor(1,9);
		LCD_20x4_Print("ID:%.0f", PageID);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		HAL_Delay(1000);
	} 
	else if (p == FINGERPRINT_PACKETRECIEVEERR) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Communication error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
	} 
	else if (p == FINGERPRINT_BADLOCATION) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
	} 
	else if (p == FINGERPRINT_FLASHERR) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
	} 
	else {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		return p;
	}
	HAL_Delay(100);
	return 1;
}

*/
void check_password(){
	if(key_pad_value != NULL){
		if(key_pad_value){
			password_input[dir] = key_pad_value;
			LCD_20x4_SetCursor(2,(4+dir));
			LCD_20x4_Write_Data(password_input[dir]);
			HAL_Delay(300);
			LCD_20x4_SetCursor(2,(4+dir));
			LCD_20x4_Send_String("*",STR_NOSLIDE);
			dir++;
		}
	}
	if (dir == 4){
		if(strncmp(password_input, password_setup, 4) == 0){
			pass_correct = 1;
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(2,4);
			LCD_20x4_Send_String("ACCESS GRANTED",STR_NOSLIDE);
			HAL_Delay(1000);
		}
		else {
			pass_correct = 0;
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(2,4);
			LCD_20x4_Send_String("ACCESS DENIED!",STR_NOSLIDE);
			HAL_Delay(1000);
		}
	}
}

void verify_password(){
	key_pad_value = read_keypad();
	HAL_Delay(10);
	if(key_pad_value == 'A'){
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String("  Moi dat van tay",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String("D: Back. ",STR_NOSLIDE);	
		long time_start_A = HAL_GetTick();
		while(1)
		{
			test = FingerPrintFast();
			key_pad_value = read_keypad();
			if (test == 1)
			{	
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Opened!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,10);
				LCD_20x4_Print("ID:%.0f", returnFingerID());	
				HAL_Delay(1000);
				break;
			}
			if ((key_pad_value == 'D') || (HAL_GetTick() - time_start_A >= 30000)){
				break;
			}	
		}
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
		LCD_20x4_SetCursor(3,1);
		LCD_20x4_Send_String("C: Setting",STR_NOSLIDE);
	}
	if(key_pad_value == 'B'){
		/* using password */
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String(" Enter Password",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,4);
		LCD_20x4_Send_String("####",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String("D: Back. ",STR_NOSLIDE);	
		long time_start_B = HAL_GetTick();
		dir = 0; 
		while(dir < 4)
		{
			key_pad_value = read_keypad();
			check_password();
			if ((key_pad_value == 'D') || (HAL_GetTick() - time_start_B >= 30000)){
				break;
			}		
		}
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
		LCD_20x4_SetCursor(3,1);
		LCD_20x4_Send_String("C: Setting",STR_NOSLIDE);
	}
	if(key_pad_value == 'C'){
		/* Setting */
		pass_correct = 0;
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String(" Enter Password",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,4);
		LCD_20x4_Send_String("####",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String("D: Back. ",STR_NOSLIDE);	
		long time_start_C = HAL_GetTick();
		dir = 0; 
		while(dir < 4)
		{
			key_pad_value = read_keypad();
			check_password();
			if ((key_pad_value == 'D') || (HAL_GetTick() - time_start_C >= 30000)){
				break;
			}		
		}
		if (pass_correct == 1){
			LCD_20x4_Clear();
			LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String("SETTING:",STR_NOSLIDE);
			LCD_20x4_SetCursor(2,1);
			LCD_20x4_Send_String("A=Enroll",STR_NOSLIDE);
			LCD_20x4_SetCursor(3,1);
			LCD_20x4_Send_String("B=Change Password",STR_NOSLIDE);
			LCD_20x4_SetCursor(4,1);
			LCD_20x4_Send_String("C=Delete     D=Close",STR_NOSLIDE);
			while (1)
			{
				key_pad_value = read_keypad();
				if (key_pad_value == 'D'|| (HAL_GetTick() - time_start_C >= 30000)){
					break;
				}
				if (key_pad_value == 'A'){
					LCD_20x4_Clear();
					LCD_20x4_SetCursor(1,1);
					LCD_20x4_Send_String("enroll finger",STR_NOSLIDE);
					HAL_Delay(1000);
					//enroll function
					break;
				}
				if (key_pad_value == 'B'){
					LCD_20x4_Clear();
					LCD_20x4_SetCursor(1,1);
					LCD_20x4_Send_String("change password",STR_NOSLIDE);
					HAL_Delay(1000);
					change_password();
					break;
				}
				if (key_pad_value == 'C'){
					LCD_20x4_Clear();
					LCD_20x4_SetCursor(1,1);
					LCD_20x4_Send_String("delete finger",STR_NOSLIDE);
					HAL_Delay(1000);
					//delete finger id
					break;
				}	
			}
		}
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
		LCD_20x4_SetCursor(3,1);
		LCD_20x4_Send_String("C: Setting",STR_NOSLIDE);		
	}
}

