#include <WiFi.h>
const char* ssid="Flybox_2BC1";
const char* passwd="THmnFcnRTJHd";
void setup() {
  Serial.begin(9600);
  Serial.println("---Programm start ---");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,passwd);
  //Serial.print("hello");
  while(!(WiFi.status()==WL_CONNECTED)){
    Serial.print("connecting.");
    delay(300);
  }
  Serial.println("---connected---");
  Serial.println("your IP is:");
  Serial.println(WiFi.localIP());
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
