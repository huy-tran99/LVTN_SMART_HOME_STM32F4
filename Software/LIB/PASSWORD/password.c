#include "password.h"

char key_pad_value;

char password_setup[4] = "1234";
char password_input[4];
char password_change[4];
char password_confirm[4];
char finger_id[3];
char finger_id_delete[3];

int dir = 0;
int pass_correct = 0;

void open_door(void)
{
	music_play(800);
	HAL_Delay(500);
	music_play(400);
	HAL_Delay(500);
	music_stop();
	servo_position(1, 70);
	HAL_Delay(100);
}

void close_door(void)
{
	music_play(400);
	HAL_Delay(500);
	music_play(800);
	HAL_Delay(500);
	music_stop();
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
		if((passwordkey_change != NULL) && (passwordkey_change != 'D')){
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
		if((passwordkey_confirm != NULL) && (passwordkey_confirm != 'D')){
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

uint8_t get_userfingerid_delete(void){
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String(" Delete ID ",STR_NOSLIDE);
	LCD_20x4_SetCursor(2,1);
	LCD_20x4_Send_String(" ID 1 - 255:",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel. ",STR_NOSLIDE);
	int l = 0;
	while (l < 3)
	{
		char key_fingerid_delete =  read_keypad();
		if((key_fingerid_delete != NULL) && (key_fingerid_delete != 'D')){
			if(key_fingerid_delete){
				finger_id_delete[l] = key_fingerid_delete;
				LCD_20x4_SetCursor(2,(15+l));
				LCD_20x4_Write_Data(finger_id_delete[l]);
				HAL_Delay(300);
				l++;
			}
		}
		else if(key_fingerid_delete == 'D'){
			break;
		}
	}
	if (l == 3){
		uint16_t _finger_id_delete = (finger_id_delete[0]-48)*100 + (finger_id_delete[1]-48)*10 + (finger_id_delete[2]-48);
		if ((_finger_id_delete > 255)||(_finger_id_delete < 1)){
			return 0;
		}
		return _finger_id_delete;
	}
	return 0;
}

uint8_t get_userfingerid(void){
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String(" Enter Finger ID ",STR_NOSLIDE);
	LCD_20x4_SetCursor(2,1);
	LCD_20x4_Send_String(" ID 1 - 255:",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel. ",STR_NOSLIDE);
	int k = 0;
	while (k < 3)
	{
		char key_fingerid =  read_keypad();
		if((key_fingerid != NULL) && (key_fingerid != 'D')){
			if(key_fingerid){
				finger_id[k] = key_fingerid;
				LCD_20x4_SetCursor(2,(15+k));
				LCD_20x4_Write_Data(finger_id[k]);
				HAL_Delay(300);
				k++;
			}
		}
		else if(key_fingerid == 'D'){
			break;
		}
	}
	if (k == 3){
		uint16_t _finger_id = (finger_id[0]-48)*100 + (finger_id[1]-48)*10 + (finger_id[2]-48);
		if ((_finger_id > 255)||(_finger_id < 1)){
			return 0;
		}
		return _finger_id;
	}
	return 0;
}

/* Function to delete finger */
void finger_delete(uint8_t FingerIdDelete){
	int p = -1;
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String("Waiting to delete",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel. ",STR_NOSLIDE);	
	
	while (p != FINGERPRINT_OK)
	{
		p = deleteModel(FingerIdDelete);
		char key_delete_finger = read_keypad();
		if (key_delete_finger == 'D')
		   break;
	}
	//char DeleteID[20];
	//sprintf(DeleteID, "Delete ID %d", FingerIdDelete);
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Print("Delete ID: %.0f", FingerIdDelete);	
	//LCD_20x4_Send_String(DeleteID,STR_NOSLIDE);
	HAL_Delay(1000);
}

/* Function to enroll finger */
int finger_enroll(uint8_t FingerId)
{	
	LCD_20x4_Clear();
	LCD_20x4_SetCursor(1,1);
	LCD_20x4_Send_String(" Put your finger ",STR_NOSLIDE);
	LCD_20x4_SetCursor(4,1);
	LCD_20x4_Send_String("D: Cancel. ",STR_NOSLIDE);	
	int p = -1;
	while (p != FINGERPRINT_OK)
   	{
		char key_pad_enroll = read_keypad();
		if (key_pad_enroll == 'D')
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
				HAL_Delay(300);
				break;
			case FINGERPRINT_NOFINGER:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Take Image 1",STR_NOSLIDE);
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
				HAL_Delay(300);
				break;
			case FINGERPRINT_IMAGEFAIL:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Imaging error",STR_NOSLIDE);
				HAL_Delay(300);
				break;
			default:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
				HAL_Delay(300);
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
			HAL_Delay(300);
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
		char key_pad_enroll = read_keypad();
		if (key_pad_enroll == 'D')
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
				HAL_Delay(300);
				break;
         	case FINGERPRINT_NOFINGER:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Take Image 2",STR_NOSLIDE);
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
				HAL_Delay(300);
				break;
			case FINGERPRINT_IMAGEFAIL:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Imaging error",STR_NOSLIDE);
				HAL_Delay(300);
				break;
			default:
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(3,1);
				LCD_20x4_Send_String("Hold D:",STR_NOSLIDE);
				LCD_20x4_SetCursor(4,1);
				LCD_20x4_Send_String("Cancel and close!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
				HAL_Delay(300);
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
			HAL_Delay(300);
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
		HAL_Delay(300);
		return p;
   	} 
	else if (p == FINGERPRINT_ENROLLMISMATCH) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Not match",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		HAL_Delay(300);
		return p;
	} 
	else {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		HAL_Delay(300);
		return p;
	}

	p = storeModel(FingerId);
	HAL_Delay(500);	
	if (p == FINGERPRINT_OK) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Stored ",STR_NOSLIDE);
		LCD_20x4_SetCursor(1,9);
		LCD_20x4_Print("ID:%.0f", FingerId);
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
		HAL_Delay(1000);
		return p;
	} 
	else if (p == FINGERPRINT_BADLOCATION) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		HAL_Delay(1000);
		return p;
	} 
	else if (p == FINGERPRINT_FLASHERR) {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		HAL_Delay(1000);
		return p;
	} 
	else {
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
		HAL_Delay(1000);
		return p;
	}
	return 1;
}

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
		LCD_20x4_Send_String("  Put your finger",STR_NOSLIDE);
		LCD_20x4_SetCursor(4,1);
		LCD_20x4_Send_String("D: Back. ",STR_NOSLIDE);	
		long time_start_A = HAL_GetTick();
		while(1)
		{
			uint8_t test = FingerPrintFast();
			key_pad_value = read_keypad();
			if (test == 1)
			{	
				LCD_20x4_Clear();
				LCD_20x4_SetCursor(1,1);
				LCD_20x4_Send_String ("Opened!",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,10);
				LCD_20x4_Print("ID:%.0f", returnFingerID());	
				open_door();
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
		LCD_20x4_Send_String("C: Cai dat",STR_NOSLIDE);
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
		if (pass_correct == 1){
			open_door();
		}
		else{
			close_door();
		}
		LCD_20x4_Clear();
		LCD_20x4_SetCursor(1,1);
		LCD_20x4_Send_String("A: Quet van tay",STR_NOSLIDE);
		LCD_20x4_SetCursor(2,1);
		LCD_20x4_Send_String("B: Nhap mat khau",STR_NOSLIDE);
		LCD_20x4_SetCursor(3,1);
		LCD_20x4_Send_String("C: Cai dat",STR_NOSLIDE);
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
					uint8_t FingerID = get_userfingerid();
					if (FingerID == 0){
						break;
					}
					finger_enroll(FingerID);
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
					uint8_t FingerIDDelete = get_userfingerid_delete();
					if (FingerIDDelete == 0){
						break;
					}
					finger_delete(FingerIDDelete);
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
		LCD_20x4_Send_String("C: Cai dat",STR_NOSLIDE);		
	}
}

