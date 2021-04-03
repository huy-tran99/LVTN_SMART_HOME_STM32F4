  lcd_send_cmd (0x80|0x00);     //dong 1
  lcd_send_string("HELLO WORLD");

  lcd_send_cmd (0x80|0x40);	//dong 2		
  lcd_send_string("LCD 20x4 DEMO");

  lcd_send_cmd (0x80|0x14);	//dong 3
  lcd_send_string("BY");

  lcd_send_cmd (0x80|0x54);	//dong 4
  lcd_send_string("ControllersTech");
_________________________________________________________________

  lcd_send_cmd (0x1C);  // display shift right
  HAL_Delay(350);
  lcd_send_cmd (0x18);  // display shift left
  HAL_Delay(350);


