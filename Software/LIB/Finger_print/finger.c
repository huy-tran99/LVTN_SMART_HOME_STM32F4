#include "finger.h"
#include "stm32f4xx_hal.h"
#include "string.h"

//#########################################################################################################################
extern UART_HandleTypeDef FINGERPRINT_USART;
extern uint8_t txBuf[32];
extern uint8_t rxBuf[32];

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
   temp = Finger.RxBuffer[len-7];
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
		HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,13,100);
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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,100);

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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,100);
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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,15,100);
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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,17,100);
		HAL_Delay(100);
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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,17,100);
		HAL_Delay(100);
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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,12,100);

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
   HAL_UART_Transmit(&FINGERPRINT_USART,Finger.TxBuffer,16,100);

   return receive_finger(12);
   /** 
      confirmation == 00H: empty success
      confirmation == 01H: error when receiving package
      confirmation == 11H: fail to clear finger library
   **/  
}
