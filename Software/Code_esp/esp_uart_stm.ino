#include <SoftwareSerial.h>

#define D3 (0)
#define D4 (2)

SoftwareSerial Test;

void setup() {
  Serial.begin(115200);
  Test.begin(115200, SWSERIAL_8N1, D3, D4, false, 256);
}

// the loop function runs over and over again forever
void loop() {
  Test.print("HELLO\n");
  delay(100);
  Test.print("HI\n");
  delay(100);
  if (Serial.available()) {
    String data_receice = Serial.readStringUntil('\n');
    Serial.println(data_receice);
  }
  delay(100);
}
