#include "finger.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "lcd_20x4.h"
#include "lib_keypad.h"
//#########################################################################################################################
extern UART_HandleTypeDef FINGERPRINT_USART;
extern uint8_t txBuf[32];
extern uint8_t rxBuf[32];
extern uint8_t rxIndex;
extern uint8_t PageID;
extern uint8_t key;

int FingerID = 0;

Finger_t	Finger;

//#########################################################################################################################
uint8_t receive_finger(uint8_t len)
{
   int i = 0; uint8_t temp;
   memset(Finger.RxBuffer,0,sizeof(Finger.RxBuffer));
   while(i < len){
      Finger.RxBuffer[i] = rxBuf[i];
      i++;  
   }
   temp = Finger.RxBuffer[len-3];
	 HAL_Delay(10);
   return temp;
}
//#########################################################################################################################
uint8_t receive_finger_match(uint8_t len)
{
   int i = 0; uint8_t temp;
   memset(Finger.RxBuffer,0,sizeof(Finger.RxBuffer));
   while(i < len){
      Finger.RxBuffer[i] = rxBuf[i];
      i++;  
   }
   temp = Finger.RxBuffer[len-5];
	 HAL_Delay(10);
   return temp;
}
uint8_t receive_finger_search(uint8_t len)
{
   int i = 0; uint8_t temp;
   memset(Finger.RxBuffer,0,sizeof(Finger.RxBuffer));
   while(i < len){
      Finger.RxBuffer[i] = rxBuf[i];
      i++;  
   }
		FingerID = Finger.RxBuffer[len-5];
   temp = Finger.RxBuffer[len-7];
	 HAL_Delay(10);
   return temp;
}

uint8_t verifyPassword(void)
{
		memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
		Finger.TxBuffer[0]=0xEF;
		Finger.TxBuffer[1]=0x01;
		Finger.TxBuffer[2]=0xFF;
		Finger.TxBuffer[3]=0xFF;
		Finger.TxBuffer[4]=0xFF;
		Finger.TxBuffer[5]=0xFF;
		Finger.TxBuffer[6]=0x01;
		Finger.TxBuffer[7]=0x00;
		Finger.TxBuffer[8]=0x07;
		Finger.TxBuffer[9]=0x13;
		Finger.TxBuffer[10]=0x00;
		Finger.TxBuffer[11]=0x00;
      Finger.TxBuffer[12]=0x00;
		Finger.TxBuffer[13]=0x00;
      Finger.TxBuffer[14] = 0x00; 
	   Finger.TxBuffer[15] = 0x1B; 

		HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,16,200);
		rxIndex = 0;
		HAL_Delay(300);
		return receive_finger(12);
   /** 
      confirmation == 00H: Correct password; 
      confirmation == 01H: error when receiving package;
      confirmation == 13H: Wrong password; 
   **/
}

uint8_t getImage(void)
{
		memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
		Finger.TxBuffer[0]=0xEF;
		Finger.TxBuffer[1]=0x01;
		Finger.TxBuffer[2]=0xFF;
		Finger.TxBuffer[3]=0xFF;
		Finger.TxBuffer[4]=0xFF;
		Finger.TxBuffer[5]=0xFF;
		Finger.TxBuffer[6]=0x01;
		Finger.TxBuffer[7]=0x00;
		Finger.TxBuffer[8]=0x03;
		Finger.TxBuffer[9]=0x01;
		Finger.TxBuffer[10]=0x00;
		Finger.TxBuffer[11]=0x05;
		HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,200);
		rxIndex = 0;
		HAL_Delay(300);
		return receive_finger(12);
   /** 
      confirmation == 00H: finger collection success
      confirmation == 01H: error when receiving package
      confirmation == 02H: can't detect finger
      confirmation == 03H: fail to collect finger
   **/
}

uint8_t image2Tz(uint8_t slot)
{//ghi du lieu van tay vao bo nho dem local(local co the la: 0x01 vung 1, 0x02 vung 2)
      memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
		int  sum = 0x00;
		sum = slot + 0x07;
		Finger.TxBuffer[0]=0xEF;
		Finger.TxBuffer[1]=0x01;
		Finger.TxBuffer[2]=0xFF;
		Finger.TxBuffer[3]=0xFF;
		Finger.TxBuffer[4]=0xFF;
		Finger.TxBuffer[5]=0xFF;
		Finger.TxBuffer[6]=0x01;
		Finger.TxBuffer[7]=0x00;
		Finger.TxBuffer[8]=0x04;
		Finger.TxBuffer[9]=0x02;
		Finger.TxBuffer[10]=slot;
		Finger.TxBuffer[11]=0x00;
		Finger.TxBuffer[12]=sum;
		HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,13,200);
		rxIndex = 0;
		HAL_Delay(300);
		return receive_finger(12);
   /** 
      confirmation == 00H: generate character file complete
      confirmation == 01H: error when receiving package
      confirmation == 06H: fail to generate character file due to the over-disorderly fingerprint image
      confirmation == 07H: fail to generate character file due to lackness point or over-smallness of fingerprint image
      confirmation == 15H: fail to generate the image for the lackness of valid primary image
   **/
}


uint8_t match(void)
{  //so sanh 2 bo dem ve trung khop van tay
   memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
   Finger.TxBuffer[0]=0xEF;
   Finger.TxBuffer[1]=0x01;
   Finger.TxBuffer[2]=0xFF;
   Finger.TxBuffer[3]=0xFF;
   Finger.TxBuffer[4]=0xFF;
   Finger.TxBuffer[5]=0xFF;
   Finger.TxBuffer[6]=0x01;
   Finger.TxBuffer[7]=0x00;
   Finger.TxBuffer[8]=0x03;
   Finger.TxBuffer[9]=0x03;
   Finger.TxBuffer[10]=0x00;
   Finger.TxBuffer[11]=0x07;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,200);
	 rxIndex = 0;
	 HAL_Delay(300);
   return receive_finger_match(14);
   /** 
      confirmation == 00H: templates of the two buffers are matching!
      confirmation == 01H: error when receiving package
      confirmation == 08H: templates of the two buffers aren't matching!
   **/
}

uint8_t createModel(void)
{  //tao ma van tay chuan tu 2 bo dem
   memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
   Finger.TxBuffer[0]=0xEF;
	Finger.TxBuffer[1]=0x01;
	Finger.TxBuffer[2]=0xFF;
	Finger.TxBuffer[3]=0xFF;
	Finger.TxBuffer[4]=0xFF;
	Finger.TxBuffer[5]=0xFF;
	Finger.TxBuffer[6]=0x01;
	Finger.TxBuffer[7]=0x00;
	Finger.TxBuffer[8]=0x03;
	Finger.TxBuffer[9]=0x05;
	Finger.TxBuffer[10]=0x00;
	Finger.TxBuffer[11]=0x09;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,200);
	rxIndex = 0;
	HAL_Delay(300);
   return receive_finger(12);
   /** 
      confirmation == 00H: operation success
      confirmation == 01H: error when receiving package
      confirmation == 0aH: fail to combine the character files. That's, the character files don't belong to one finger
   **/
}

uint8_t storeModel(uint8_t PageID)
{  // luu ma van tay chuan vao flash
   memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
   uint8_t sum1;
   sum1= 0x0E + PageID;
   Finger.TxBuffer[0]=0xEF;
	Finger.TxBuffer[1]=0x01;
	Finger.TxBuffer[2]=0xFF;
	Finger.TxBuffer[3]=0xFF;
	Finger.TxBuffer[4]=0xFF;
	Finger.TxBuffer[5]=0xFF;
	Finger.TxBuffer[6]=0x01;
	Finger.TxBuffer[7]=0x00;
	Finger.TxBuffer[8]=0x06;
	Finger.TxBuffer[9]=0x06;
	Finger.TxBuffer[10]=0x01;
	Finger.TxBuffer[11]=0x00;
   Finger.TxBuffer[12]=PageID;
	Finger.TxBuffer[13]=0x00;
	Finger.TxBuffer[14]=sum1;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,15,200);
	rxIndex = 0;
	HAL_Delay(300);
   return receive_finger(12);
   /** 
      confirmation == 00H: storage success
      confirmation == 01H: error when receiving package
      confirmation == 0bH: addressing PageID is beyond the finger library
      confirmation == 18H: eror when writing Flash
   **/
}

uint8_t search(void)
{  //lay ma van tay chua tu flash ra de so sanh voi van tay vua nhan tren bo dem
   memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
   Finger.TxBuffer[0]=0xEF;
	Finger.TxBuffer[1]=0x01;
	Finger.TxBuffer[2]=0xFF;
	Finger.TxBuffer[3]=0xFF;
	Finger.TxBuffer[4]=0xFF;
	Finger.TxBuffer[5]=0xFF;
   //check sum duoc tinh tu day
	Finger.TxBuffer[6]=0x01;
	Finger.TxBuffer[7]=0x00;
	Finger.TxBuffer[8]=0x08;
	Finger.TxBuffer[9]=0x04;
	Finger.TxBuffer[10]=0x01;
   //dia chi bat dau
	Finger.TxBuffer[11]=0x00;
   Finger.TxBuffer[12]=0x00;
   //dia chi ket thuc
	Finger.TxBuffer[13]=0x00;
	Finger.TxBuffer[14]=0xff;
   //tinh toan check sum
   Finger.TxBuffer[15]=0x01;
   Finger.TxBuffer[16]=0x0D;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,17,200);
		rxIndex = 0;
		HAL_Delay(300);
	
   return receive_finger_search(16);
   /** 
      confirmation == 00H: found the matching finer
      confirmation == 01H: error when receiving package
      confirmation == 09H: no matching
   **/
}
uint8_t search_master(void)
{  //kiem tra ma van tay dat vao co phai la van tay goc hay khong
   memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
   Finger.TxBuffer[0]=0xEF;
	Finger.TxBuffer[1]=0x01;
	Finger.TxBuffer[2]=0xFF;
	Finger.TxBuffer[3]=0xFF;
	Finger.TxBuffer[4]=0xFF;
	Finger.TxBuffer[5]=0xFF;
   //check sum duoc tinh tu day
	Finger.TxBuffer[6]=0x01;
	Finger.TxBuffer[7]=0x00;
	Finger.TxBuffer[8]=0x08;
	Finger.TxBuffer[9]=0x04;
	Finger.TxBuffer[10]=0x01;
   //dia chi bat dau
	Finger.TxBuffer[11]=0x00;
  Finger.TxBuffer[12]=0x00;
   //dia chi ket thuc
	Finger.TxBuffer[13]=0x00;
	Finger.TxBuffer[14]=0x01;
   //tinh toan check sum
   Finger.TxBuffer[15]=0x00;
   Finger.TxBuffer[16]=0x0F;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,17,200);
	 rxIndex = 0;
	HAL_Delay(300);
   return receive_finger_search(16);
   /** 
      confirmation == 00H: found the matching finer
      confirmation == 01H: error when receiving package
      confirmation == 09H: no matching
   **/   
}

uint8_t empty(void)
{	
   //to delete all the templates in the Flash library	
   memset(Finger.TxBuffer,0,sizeof(Finger.TxBuffer));
	Finger.TxBuffer[0]=0xEF;
	Finger.TxBuffer[1]=0x01;
	Finger.TxBuffer[2]=0xFF;
	Finger.TxBuffer[3]=0xFF;
	Finger.TxBuffer[4]=0xFF;
	Finger.TxBuffer[5]=0xFF;
	Finger.TxBuffer[6]=0x01;
	Finger.TxBuffer[7]=0x00;
	Finger.TxBuffer[8]=0x03;
	Finger.TxBuffer[9]=0x0D;
	Finger.TxBuffer[10]=0x00;
	Finger.TxBuffer[11]=0x11;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,200);
	rxIndex = 0;
	HAL_Delay(500);
   return receive_finger(12);
   /** 
      confirmation == 00H: empty success
      confirmation == 01H: error when receiving package
      confirmation == 11H: fail to clear finger library
   **/   
}

uint8_t deleteModel(uint8_t PageID)
{
   //to delete a segment (N) of templates of Flash library started from the specified location (or PageID)
   
   uint8_t sum2;
   sum2= 0x15 + PageID;
   Finger.TxBuffer[0]=0xEF;
   Finger.TxBuffer[1]=0x01;
   Finger.TxBuffer[2]=0xFF;
   Finger.TxBuffer[3]=0xFF;
   Finger.TxBuffer[4]=0xFF;
   Finger.TxBuffer[5]=0xFF;
   Finger.TxBuffer[6]=0x01;
   Finger.TxBuffer[7]=0x00;
   Finger.TxBuffer[8]=0x07;
   Finger.TxBuffer[9]=0x0c;
   Finger.TxBuffer[10]=0x00;
   Finger.TxBuffer[11]=PageID;
   Finger.TxBuffer[12]=0x00;
   Finger.TxBuffer[13]=0x01;
   Finger.TxBuffer[14]=0x00;
   Finger.TxBuffer[15]=sum2;
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,16,200);
	rxIndex = 0;
	HAL_Delay(500);
   return receive_finger(12);
   /** 
      confirmation == 00H: empty success
      confirmation == 01H: error when receiving package
      confirmation == 11H: fail to clear finger library
   **/  
}

uint8_t returnFingerID(void)
{
   return FingerID;
}


//##########################################################################
uint8_t FingerPrintFast(void){
    uint8_t p = getImage();
		HAL_Delay(50);
		if (p != FINGERPRINT_OK)  return -1;
		HAL_Delay(50);

    p = image2Tz(1);
		HAL_Delay(50);
		if (p != FINGERPRINT_OK)  return -1;
		HAL_Delay(50);
		p=search();
		HAL_Delay(100);
		if (p != FINGERPRINT_OK)  {HAL_Delay(300); return -1;}
		
    // found a match!
    LCD_20x4_Clear();
    LCD_20x4_SetCursor(1,1);
    LCD_20x4_Send_String ("Opened!",STR_NOSLIDE);
		
		
    LCD_20x4_SetCursor(1,10);
    LCD_20x4_Print("ID:%.0f", returnFingerID());
		
		HAL_Delay(100);
    return 1;
}




uint8_t Enroll(void)
{	
//	key = read_keypad();////////////////////////////////////
//	if (key == 'D')/////////////////////////////////////////////
//	return 0;///////////////////////////////////
	int p = -1;
	while (p != FINGERPRINT_OK)
    {
		key = read_keypad();///////////////////////////////////
		if (key == 'D')//////////////////////////////////
		break;/////////////////////////////////////
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
      //Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      //Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      //Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      //Serial.println("Could not find fingerprint features");
      return p;
    default:
      //Serial.println("Unknown error");
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
	key = read_keypad();////////////////////////////////////////////////////
	if (key == 'D')//////////////////////////////////////////////////
	break;//////////////////////////////////////////////////////////
    p = getImage();
		HAL_Delay(500);
	
}

p = -1;
    while (p != FINGERPRINT_OK) 
    {	
			key = read_keypad();//////////////////////////////////////////////////////
			if (key == 'D')/////////////////////////////////////////////////////
			break;/////////////////////////////////////////////////////////////
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

//OK

p = image2Tz(2);
HAL_Delay(500);		
switch (p) 
{
    case FINGERPRINT_OK:
			LCD_20x4_Clear();
      LCD_20x4_SetCursor(1,1);
			LCD_20x4_Send_String ("Character2",STR_NOSLIDE);
			HAL_Delay(1000);
      //Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      //Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      //Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      //Serial.println("Could not find fingerprint features");
      return p;
    default:
      //Serial.println("Unknown error");
      return p;
}
// OK converted!
//Serial.print("Creating model for #");  Serial.println(id);

p = createModel();
HAL_Delay(500);
    if (p == FINGERPRINT_OK) 
    {
        //Serial.println("Prints matched!");
    } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
        //Serial.println("Communication error");
        LCD_20x4_Clear();
        LCD_20x4_SetCursor(1,1);
        LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
        LCD_20x4_SetCursor(4,1);
        LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
        return p;
    } else if (p == FINGERPRINT_ENROLLMISMATCH) {
        //Serial.println("Fingerprints did not match");
        LCD_20x4_Clear();
        LCD_20x4_SetCursor(1,1);
        LCD_20x4_Send_String ("Not match",STR_NOSLIDE);
        LCD_20x4_SetCursor(4,1);
        LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
        return p;
    } else {
        //Serial.println("Unknown error ");
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
        //Serial.println("Stored!");
        LCD_20x4_Clear();
        LCD_20x4_SetCursor(1,1);
        LCD_20x4_Send_String ("Stored ",STR_NOSLIDE);
				LCD_20x4_SetCursor(1,9);
				LCD_20x4_Print("ID:%.0f", PageID);
        LCD_20x4_SetCursor(4,1);
        LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
        HAL_Delay(1000);
    } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
        //Serial.println("Communication error");
        LCD_20x4_Clear();
        LCD_20x4_SetCursor(1,1);
        LCD_20x4_Send_String ("Communication error",STR_NOSLIDE);
        LCD_20x4_SetCursor(4,1);
        LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
        return p;
    } else if (p == FINGERPRINT_BADLOCATION) {
        //Serial.println("Could not store in that location");
        LCD_20x4_Clear();
        LCD_20x4_SetCursor(1,1);
        LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
        LCD_20x4_SetCursor(4,1);
        LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
        return p;
    } else if (p == FINGERPRINT_FLASHERR) {
        //Serial.println("Error writing to flash");
        LCD_20x4_Clear();
        LCD_20x4_SetCursor(1,1);
        LCD_20x4_Send_String ("Unknown error",STR_NOSLIDE);
        LCD_20x4_SetCursor(4,1);
        LCD_20x4_Send_String ("             D=Close",STR_NOSLIDE);
        return p;
    } else {
        //Serial.println("Unknown error");
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


uint8_t getFingerprintID(void) 
{
    uint8_t p = getImage();
	HAL_Delay(500);
    switch (p) {
        case FINGERPRINT_OK:
        //Serial.println("Image taken");
        break;
        case FINGERPRINT_NOFINGER:
        //Serial.println("No finger detected");
        return p;
        case FINGERPRINT_PACKETRECIEVEERR:
        //Serial.println("Communication error");
        return p;
        case FINGERPRINT_IMAGEFAIL:
        //Serial.println("Imaging error");
        return p;
        default:
        //Serial.println("Unknown error");
        return p;
    }
    p = image2Tz(1);
		HAL_Delay(500);
    switch (p) {
        case FINGERPRINT_OK:
        //Serial.println("Image converted");
        break;
        case FINGERPRINT_IMAGEMESS:
        //Serial.println("Image too messy");
        return p;
        case FINGERPRINT_PACKETRECIEVEERR:
        //Serial.println("Communication error");
        return p;
        case FINGERPRINT_FEATUREFAIL:
        //Serial.println("Could not find fingerprint features");
        return p;
        case FINGERPRINT_INVALIDIMAGE:
        //Serial.println("Could not find fingerprint features");
        return p;
        default:
        //Serial.println("Unknown error");
        return p;
    }

    p = search();
		HAL_Delay(1000);
    if (p == FINGERPRINT_OK) {
        //Serial.println("Found a print match!");
    } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
        //Serial.println("Communication error");
        return p;
    } else if (p == FINGERPRINT_NOTFOUND) {
        //Serial.println("Did not find a match");
        return p;
    } else {
        //Serial.println("Unknown error");
        return p;
    }

    LCD_20x4_Clear();
    LCD_20x4_SetCursor(1,10);
    LCD_20x4_Print("ID:%.0f", returnFingerID());
    return rxBuf[11];
}


