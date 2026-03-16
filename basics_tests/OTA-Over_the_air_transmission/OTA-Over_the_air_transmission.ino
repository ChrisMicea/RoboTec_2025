#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "Christian's Phone";
const char* password = "dkdl4589";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi Connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());

  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
}