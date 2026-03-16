const int trigPin = 4;  
const int echoPin = 5;  

float duration = 0; 
float distance = 0;

void setup() {
  Serial.begin(115200);  
  // Serial.print("Go fuck yourself");
  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);  
}

void loop() {
  // Trigger the sensor
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trigPin, LOW); 

  // Measure the echo pulse duration
  duration = pulseIn(echoPin, HIGH);  
  distance = (duration * 0.0343) / 2;  // Calculate distance in cm

  Serial.print("Distance: ");  
  Serial.println(distance);  
  delay(100);  // Adjust delay as needed
}