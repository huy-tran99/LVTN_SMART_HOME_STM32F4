void DHT11() {
  /*Only write*/
  /* DHT11: Nhiệt độ và độ ẩm */
  char *arg1;
  char *arg2;
  arg1 = CMD.next();
  Firebase.setString("sensor/DHT11/temp", arg1);
  arg2 = CMD.next();
  Firebase.setString("sensor/DHT11/humi", arg2);
}

void PIR() {
  /*Only write*/
  /*PIR: 1 tham so la tat hoac mo */
  char *arg;
  arg = CMD.next();
  Firebase.setString("sensor/PIR", arg);
}

void RAIN() {
  /*Only write*/
  /* RAIN 0 hoặc 1 */
  char *arg;
  arg = CMD.next();
  Firebase.setString("sensor/RAIN", arg);
}

void light_sensor() {
  /*Only write*/
  /* light 0 hoặc 1 */
  char *arg;
  arg = CMD.next();
  Firebase.setString("sensor/LIGHT", arg);
}

void GAS() {
  /*Only write*/
  /* GAS 0 hoặc 1 */
  char *arg;
  arg = CMD.next();
  Firebase.setString("sensor/GAS", arg);
}

void LED() {
  //2 tham so la vi tri led va trang thai led
  //read and write
  char *arg1;
  char *arg2;
  int val;
  arg1 = CMD.next();
  arg2 = CMD.next();
  val = atoi(arg1);
  if (val == 1) {
    Serial.println("LED 1:");
    //Firebase.setString("stream/light/led1", arg2);
  }
  if (val == 2) {
    Serial.println("LED 2:");
    //Firebase.setString("stream/light/led2", arg2);
  }
}

void security() {
  /* read and write */
  /* control door */
  char *arg;
  arg = CMD.next();
  Firebase.setString("stream/security/door", arg);
}

void relay() {
  /* read and write */
  /* control relay */
  char *arg;
  arg = CMD.next();
  Firebase.setString("stream/device/relay", arg);
}

void fan() {
  /* read and write */
  /* control fan */
  char *arg;
  arg = CMD.next();
  Firebase.setString("stream/device/fan", arg);
}

void erase() {
  /* To delete ssid and password in ROM */
  // write a 0 to 96 bytes of the EEPROM
  for (int i = 0; i < 96; ++i) {
    EEPROM.write(i, 0);
  }
  ESP.reset();
}
