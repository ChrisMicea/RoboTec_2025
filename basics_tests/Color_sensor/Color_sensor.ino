//---------color sensor that returns 0-white, 1-black-------
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Create a Wire instance with custom SDA/SCL
TwoWire I2CBus = TwoWire(0);  // use 0 or 1 depending on ESP32 hardware


Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS, 
  TCS34725_GAIN_4X
);

const int ledPin = 35;

void setup()
{
  Serial.begin(115200);
  delay(100);

  I2CBus.begin(8, 9); // SDA=36, SCL=37

  // Initialize sensor with custom I2C bus
  if (!tcs.begin(TCS34725_ADDRESS, &I2CBus)) {
    Serial.println("Could not find TCS34725 sensor, check wiring!");
    while (1);
  }

  Serial.println("TCS34725 sensor found!");
  pinMode(ledPin, OUTPUT);
  analogWrite(ledPin, 128);
}

// Function that returns 1 for black, 0 for white
int detectColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);


  if (r < 55 && g < 55 && b < 55)
  {
    Serial.println("Black");
    return 1;
  }
  else
  {
    Serial.println("White");
    return 0;
  }


}

void loop()
{
  int color = detectColor();
  // Use `color` for further logic
  delay(500);
}