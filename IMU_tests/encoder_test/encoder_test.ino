#define SENSOR_PIN 8  // Connect D0 from LM393 to GPIO 15 on ESP32

volatile unsigned long gapCount = 0;

void IRAM_ATTR onGapDetected() {
  gapCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onGapDetected, FALLING); 
  // Use RISING or CHANGE if your signal works better that way
  
}

void loop() {
  Serial.print("Gap count: ");
  Serial.println(gapCount);
  delay(2000);  // Print once per 2 seconds
}