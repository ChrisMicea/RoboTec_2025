#include <Wire.h>
#include <VL53L0X.h>

#define XSHUT_1 2
#define XSHUT_2 42
#define XSHUT_3 41

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

const int rightOffset = 25; // 0 mm offset
const int leftOffset = 0; // 20 mm offset
const int frontOffset = -10; // -30 mm offset

void setup() {
  Serial.begin(115200);
  Wire.begin(47, 48);  // Default I2C pins (ESP32: SDA 21, SCL 22)

  // Set XSHUT pins as outputs
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  // Reset all sensors
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);

  // === Bring up sensor 1 ===
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  sensor1.init(true);
  sensor1.setAddress(0x30);

  // === Bring up sensor 2 ===
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  sensor2.init(true);
  sensor2.setAddress(0x31);

  // === Bring up sensor 3 ===
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  sensor3.init(true);
  sensor3.setAddress(0x32);

  // Start ranging mode
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
}

void loop() {
  Serial.print("Sensor right: ");
  Serial.print(sensor1.readRangeContinuousMillimeters() - leftOffset);
  Serial.print(" mm | Sensor front: ");
  Serial.print(sensor2.readRangeContinuousMillimeters() - frontOffset);
  Serial.print(" mm | Sensor left: ");
  Serial.print(sensor3.readRangeContinuousMillimeters() - rightOffset);
  Serial.println(" mm");

  delay(100);
}
