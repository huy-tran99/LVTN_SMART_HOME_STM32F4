#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <FirebaseArduino.h>
#include "html.h"

#include <SoftwareSerial.h>
#include <SerialCommand.h>

SerialCommand CMD;

/* Định nghĩa 2 chân uart */
#define D3 (2)
#define D4 (0)

SoftwareSerial esp;

/* Bat dau chuong trinh, esp se doc ten va mat khau duoc luu trong ROM */
/* ESP se co ket noi toi wifi do, neu ket noi thanh cong thi thuc hien chuong trinh */
/* Neu ket noi khong thanh cong, esp se vao che do STATION va cho nguoi dung nhap ten va mat khau */
const char *ssid = "HOST_STM32F4";
const char *password = "123456789";

ESP8266WebServer sv(80); //Khoi tao server ở port 80

#define FIREBASE_HOST "smarthome-41011-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "zO7fQEf2DdJ28nS3NXi5JjwXgM03RBHOR9tOKy4r"   //Không dùng xác thực nên không đổi

/* Cac bien can thiet*/
String s0 = "\n";
String s1 = "FAN";
String s2 = "RELAY";
String s3 = "DOOR";
String s4 = "LED1";
String s5 = "LED2";

int flash;
/* Bat dau chuong trinh chinh */
void setup() {
  // put your setup code here, to run once:
  EEPROM.begin(512); //Initialasing EEPROM
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);//Set up a soft access point

  /* Đọc tên và mk wifi ở EEPROM và thử kết nối */
  String ten_wifi;
  String mk_wifi;

  Serial.println();
  Serial.println("Đọc tên wifi trên EEPROM");
  for (int i = 0; i < 32; ++i)
  {
    ten_wifi += char(EEPROM.read(i));
  }
  Serial.print("Tên wifi: ");
  Serial.println(ten_wifi);
  Serial.println();
  Serial.println("Đọc mk wifi trên EEPROM: ");
  for (int i = 32; i < 96; ++i)
  {
    mk_wifi += char(EEPROM.read(i));
  }
  Serial.print("Mật khẩu: ");
  Serial.println(mk_wifi);
  WiFi.begin(ten_wifi, mk_wifi);
  /* begin software serial */
  esp.begin(115200, SWSERIAL_8N1, D3, D4, false, 256);

  if (testWiFi())
  {
    Serial.println("Kết nối thành công!! ");
    Serial.print("Địa chỉ IP: ");
    Serial.println(WiFi.localIP());
    esp.print("wifi1\n");
    delay(500);
    flash = 1;
    //sv.begin();//bắt đầu khởi động server
    Serial.println("Now connect to firebase");
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.stream("/stream");
  }
  else {
    Serial.println("Cấu hình lại điểm kết nối");
    WiFi.disconnect();
    WiFi.softAP(ssid , password);
    Serial.print("Địa chỉ IP của ESP: ");
    Serial.println(WiFi.softAPIP());
    flash = 0;
    sv.on("/", handleWebsite);
    sv.on("/caidat", cai_dat);
    sv.begin();//bắt đầu khởi động server
    esp.print("wifi2\n");
    delay(500);
  }
  /* add cmd to call */
  CMD.addCommand("LED", LED);
  CMD.addCommand("DHT11", DHT11);
  CMD.addCommand("PIR", PIR);
  CMD.addCommand("LIGHT", light_sensor);
  CMD.addCommand("GAS", GAS);
  CMD.addCommand("SECURITY", security);
  CMD.addCommand("RELAY", relay);
  CMD.addCommand("FAN", fan);
  CMD.addCommand("ERASE", erase);
}
/* --------------------------------------------------------- */
void loop() {
  // put your main code here, to run repeatedly:
  if (flash == 0) {
    sv.handleClient();
    esp.print("wifi2\n");
    delay(100);
  }
  else {
    if (!check_connect()) {
      Serial.println("Reconnect");
      esp.print("wifi0\n");
      delay(100);
      ESP.reset();
    }
    else {
      esp.print("wifi1\n");
      delay(100);
      if (Firebase.failed()) {
        Serial.println("error!!!");
      }
      CMD.readSerial();
      if (Firebase.available()) {
        FirebaseObject event = Firebase.readEvent();
        String path = event.getString("path");
        String data = event.getString("data");
        if (path == "/device/fan") {
          Serial.println(String("fan :") + data);
          String data_send = s1 + data + s0;
          Serial.println(data_send);
        }
        if (path == "/device/relay") {
          Serial.println(String("relay :") + data);
        }
        if (path == "/light/led1") {
          Serial.println(String("led1 :") + data);
        }
        if (path == "/light/led2") {
          Serial.println(String("led2 :") + data);
        }
        if (path == "/security/door") {
          Serial.println(String("door :") + data);
        }
      }
    }
  }
}
