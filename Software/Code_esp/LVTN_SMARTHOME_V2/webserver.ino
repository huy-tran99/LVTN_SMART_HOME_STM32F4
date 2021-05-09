void handleWebsite() {
  sv.send(200, "text/html", PAGE_NetworkConfiguration); //gửi dưới dạng html
}

/* Ham cai dat de luu ten va mat khau wifi vao ROM */
void cai_dat() {
  String tenwf = sv.arg("tenWiFi");
  String mk = sv.arg("matkhau");
  Serial.print("ten wifi:");
  Serial.println(tenwf);
  Serial.print("mat khau:");
  Serial.println(mk);
  /* Kiem tra ten va mk doc duoc co hay khong */
  if (tenwf.length() > 0 && mk.length() > 0) {
    Serial.println("clear EEPROM");
    // write a 0 to 96 bytes of the EEPROM
    for (int i = 0; i < 96; ++i) {
      EEPROM.write(i, 0);
    }
    delay(200);
    Serial.println("Chép tên WiFi vào EEPROM:");
    for (int i = 0; i < tenwf.length(); ++i) {
      EEPROM.write(i, tenwf[i]);
      Serial.print("viết tên: ");
      Serial.println(tenwf[i]);
    }
    Serial.println("Chép mật khẩu WiFi vào EEPROM:");
    for (int i = 0; i < mk.length(); ++i) {
      EEPROM.write(32 + i, mk[i]);
      Serial.print("viết mk: ");
      Serial.println(mk[i]);
    }
    delay(200);
    Serial.println("Commit");
    EEPROM.commit();
    delay(100);
    Serial.println("Reset");
  }
  /* Sau khi luu xong reset chuong trinh */
  ESP.reset();
}

/* Ham test ket noi wifi */
bool testWiFi(void)
{
  int c = 0;
  Serial.println("Wait to connect");
  while (c < 20) {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(500);
    Serial.print("*");
    c++;
  }
  Serial.println();
  Serial.println("Không thể kết nối vì quá thời gian chờ ");
  return false;
}

/* check connect to wifi */
bool check_connect(void)
{
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  else {
    return true;
  }
}
