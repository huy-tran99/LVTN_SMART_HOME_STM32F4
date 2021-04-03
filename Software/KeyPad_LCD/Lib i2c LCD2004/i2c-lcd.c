#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;                      // vi du dua con tro toi vitri 0,2. tuc la di chuyen den dia chi 02F (do datasheet quy dinh), luc nay muon dung lenh dduaw con tror thi phai set chan D7 cuar LCD = 1, dia chi 01F=000 0010. 
	uint8_t data_t[4];						// nen chon cmd = 1000 0010 (tuc bang 0x80|0x02) => data_u = 1000 0000,  data_l= 0010 0000
	data_u = (cmd&0xf0);					// data_t[0] = 1000 1100    data_t[1] = 1000 1000
	data_l = ((cmd<<4)&0xf0);				// data_t[2] = 0010 1100	data_t[3] = 0010 1000
	data_t[0] = data_u|0x0C;  //en=1, rs=0     0x0C = 0000 1100
	data_t[1] = data_u|0x08;  //en=0, rs=0     0x08 = 0000 1000   bit 2 (bit thu 3) la dieu khien chan EN cua LCD, EN 1->0 ddongs choots, guiwr giuwx lieu. gui 1 lan 4bit, gui 2 lan
	data_t[2] = data_l|0x0C;  //en=1, rs=0						  bit 1 khar nawng laf bit Write/Read, W/R = 0 thi Write.
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;						// muon in ki tu len LCD, tuc la gui data thi chan RS phai = 1 
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1     0x0D = 0000 1101  
	data_t[1] = data_u|0x09;  //en=0, rs=1	   0x09 = 0000 1001   bit 0 la bit RS 
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //thu dat bang 0x0F //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
