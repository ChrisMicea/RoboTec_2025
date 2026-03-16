#include "BluetoothSerial.h"

BluetoothSerial SerialBT;  // Create Bluetooth Serial object

void setup() {
  Serial.begin(115200);           // USB serial monitor
  SerialBT.begin("ESP32_BT_Test"); // Bluetooth device name
  Serial.println("Bluetooth started. Connect to 'ESP32_BT_Test'");
}

void loop() {
  String message = "Hello from ESP32 via Bluetooth!\n";
  SerialBT.print(message);   // Send via Bluetooth
  Serial.print("Sent: ");
  Serial.print(message);     // Also print to USB serial for debugging
  delay(1000);
}
