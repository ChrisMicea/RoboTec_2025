#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // SDA, SCL

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag(); // not available on MPU6500, harmless to try

  Serial.println("MPU6500 initialization done.");
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  Serial.print("aX: ");
  Serial.print(mySensor.accelX());
  Serial.print(" aY: ");
  Serial.print(mySensor.accelY());
  Serial.print(" aZ: ");
  Serial.println(mySensor.accelZ());

  Serial.print("gX: ");
  Serial.print(mySensor.gyroX());
  Serial.print(" gY: ");
  Serial.print(mySensor.gyroY());
  Serial.print(" gZ: ");
  Serial.println(mySensor.gyroZ());

  delay(500);
}