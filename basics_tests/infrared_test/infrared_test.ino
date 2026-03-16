#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 8
#define SCL_PIN 9

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C
  
  if (!sensor.init()) {
    Serial.println("Failed to detect VL53L0X!");
    while (1) {}  // Halt if sensor not found
  }
  
  sensor.setTimeout(500);  // Set timeout to 500ms - time the sensor waits for input until it times out
  sensor.startContinuous();  // Start continuous mode
  Serial.println("VL53L0X Ready!");
}

void loop() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  
  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout - Check wiring!");
  } 
  else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }
  
  delay(100);  // Short delay between readings
}

// test to see if the device was detected anywhere

// #include <Wire.h>
// void setup() {
//   Serial.begin(115200);
//   Wire.begin(8, 9);  // Or Wire.begin(SDA_PIN, SCL_PIN) for custom pins
//   Serial.println("I2C Scanner");
// }
// void loop() {
//   byte error, addr;
//   for (addr = 1; addr < 127; addr++) {
//     Wire.beginTransmission(addr);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("Found device at 0x");
//       Serial.println(addr, HEX);
//     }
//   }
//   delay(5000);
// }