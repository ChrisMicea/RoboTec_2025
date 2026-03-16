#include <Wire.h>
#include <VL53L0X.h>

// pin defines
#define SDA_PIN_1 8
#define SCL_PIN_1 9
#define MOTOR1_IN1 4
#define MOTOR1_IN2 5
#define MOTOR1_PWM 15
#define MOTOR2_IN3 6
#define MOTOR2_IN4 7
#define MOTOR2_PWM 16

VL53L0X sensor;

void setup() {
  // pin setup and serial begin
  Serial.begin(115200);

  // Wire.begin(SDA_PIN_1, SCL_PIN_1);  // Initialize I2C for first infrared sensor

  // if (!sensor.init()) {
  //   Serial.println("Failed to detect VL53L0X!");
  //   while (1) {}  // Halt if sensor not found
  // }

  // sensor.setTimeout(500);  // Set timeout to 500ms - time the sensor waits for input until it tmes out
  // sensor.startContinuous();  // Start continuous mode
  // Serial.println("VL53L0X Ready!");

  pinMode(MOTOR1_IN1,OUTPUT);
  pinMode(MOTOR1_IN2,OUTPUT);
  pinMode(MOTOR1_PWM,OUTPUT);
  pinMode(MOTOR2_IN3,OUTPUT);
  pinMode(MOTOR2_IN4,OUTPUT);
  pinMode(MOTOR2_PWM,OUTPUT);
}

void loop() {
  int powFwd = 220;
  int powTurn = 150;
  // dir : 1 = forward, 2 = backward, 3 = turn left, 4 = turn right

  uint16_t distance;
  
  // if (sensor.timeoutOccurred()) {
  //   Serial.println("Timeout - Check wiring!");
  // } 
  // else {
  // distance = sensor.readRangeContinuousMillimeters();
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" mm");
  // }

  driveForward(powFwd);
  delay(600);

  // stopMotors();
  // delay(600);

  // distance = sensor.readRangeContinuousMillimeters();
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" mm");

  // turnLeft(powTurn);
  // delay(600);
  
  // // stopMotors();
  // // delay(600);

  // distance = sensor.readRangeContinuousMillimeters();
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" mm");

  // turnRight(powTurn);
  // delay(600);
  
  // // stopMotors();
  // // delay(600);

  // distance = sensor.readRangeContinuousMillimeters();
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" mm");

  // driveBackward(powFwd);
  // delay(600);

  // stopMotors();
  // delay(600);
}

void driveForward(int pow) {
  moveMotor(1, 1, pow);
  moveMotor(2, 1, pow);
}
void driveBackward(int pow) {
  moveMotor(1, 0, pow);
  moveMotor(2, 0, pow);
}
void turnLeft(int pow) {
  moveMotor(1, 1, pow);
  moveMotor(2, 0, pow);
}
void turnRight(int pow) {
  moveMotor(1, 0, pow);
  moveMotor(2, 1, pow);
}
void stopMotors(void) {
  moveMotor(1, 1, 0);
  moveMotor(2, 1, 0);
}


void moveMotor(int motorNumber, int direction, int pwmPow) {
  // direction == 1 - forward
  // direction == 0 -backward

  if (motorNumber == 1) {
    if (direction == 1) {
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
    }
    else {
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
    }
    analogWrite(MOTOR1_PWM, pwmPow);
  }
  else if (motorNumber == 2) {
    if (direction == 1) {
      digitalWrite(MOTOR2_IN3, LOW);
      digitalWrite(MOTOR2_IN4, HIGH);
    }
    else {
      digitalWrite(MOTOR2_IN3, HIGH);
      digitalWrite(MOTOR2_IN4, LOW);
    }
    analogWrite(MOTOR2_PWM, pwmPow);
  }
  else{
    Serial.print("Error: motor nonexistent - pausing execution");
    while(1) {
      ;
    }
  }
}
