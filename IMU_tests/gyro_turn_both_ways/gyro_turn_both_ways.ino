#include <MPU9250_asukiaaa.h>
#include <PID_v1.h>
#include <Wire.h>

#define MPU_SDA_PIN 8
#define MPU_SCL_PIN 9
#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_PWM 15
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7
#define MOTOR_RIGHT_PWM 16

MPU9250_asukiaaa mpu;

double angle = 0, Setpoint, Input, Output;
bool isFirstTurn = 1;
unsigned long lastTime = 0;
// PID Setup
double Kp = 1.0, Ki = 0.01, Kd = 0.15;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// Gyro bias offsets (X, Y, Z)
float gyroBias[3] = {0, 0, 0};


void setup() {
  // setup_infrareds();

  pinMode(MOTOR_LEFT_IN1,OUTPUT);
  pinMode(MOTOR_LEFT_IN2,OUTPUT);
  pinMode(MOTOR_LEFT_PWM,OUTPUT);
  pinMode(MOTOR_RIGHT_IN3,OUTPUT);
  pinMode(MOTOR_RIGHT_IN4,OUTPUT);
  pinMode(MOTOR_RIGHT_PWM,OUTPUT);

  Serial.begin(115200);
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN); // SDA, SCL

  // pinMode(LEFT_ENCODER_PIN, INPUT);
  // pinMode(RIGHT_ENCODER_PIN, INPUT);
  // setup_encoders();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();

  calibrateGyro();

  Serial.println("MPU6500 initialization done.");
}

void loop() {
  if (isFirstTurn) {
    turnRobot(85.0); // greater offset needed for first turn - or maybe motors need an initial kick
    isFirstTurn = 0;
  }
  else {
    // Alternate between left and right turns
    static bool turnDirection = false; // true = right, false = left
    
    if (turnDirection) {
      turnRight();
    } else {
      turnLeft();
    }
    
    turnDirection = !turnDirection; // Toggle direction  }
  // delay(2500);
  }

  stopMotors();
  delay(500);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro. Keep sensor still...");
  delay(100);

  const int numSamples = 1000;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    if (mpu.gyroUpdate() == 0) { // Successfully read gyro
      sumX += mpu.gyroX();
      sumY += mpu.gyroY();
      sumZ += mpu.gyroZ();
    }
    delay(5);
  }

  // Calculate average bias (offset)
  gyroBias[0] = sumX / numSamples;
  gyroBias[1] = sumY / numSamples;
  gyroBias[2] = sumZ / numSamples;

  Serial.print("Gyro Bias (X/Y/Z): ");
  Serial.print(gyroBias[0], 4);
  Serial.print(", ");
  Serial.print(gyroBias[1], 4);
  Serial.print(", ");
  Serial.println(gyroBias[2], 4);
}


void turnRobot(float targetAngle) {
  reset_PID(); // Reset internal PID state - ensures no carry-over from previous turns

  angle = 0; // Reset angle
  
  Setpoint = normalizeAngle(targetAngle);
  lastTime = millis(); // initialize lastTime - so it deosn't start with 0, flubbing the calculations form the get-go
  unsigned long endTime = millis() + 1200; // forcefully stops the movement after 1.2 seconds 
  while (abs(normalizeAngle(angle - Setpoint)) > 2.0) { // && millis() < endTime
    updateAngle();
    Input = normalizeAngle(angle);
    myPID.Compute();

     // Apply minimum power threshold
    if (Output > 0 && Output < 80) Output = 80;
    if (Output < 0 && Output > -80) Output = -80;

    driveMotors(Output);
    delay(10);
  }
  stopMotors();
}

float normalizeAngle(float angle) {
  angle = fmod(angle, 360); // Reduce to [0, 360)
  if (angle > 180) // normaliza in the [-180, 180] range
    angle -= 360;
  return angle;
}

void updateAngle() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  mpu.gyroUpdate();
  float gyroZ = mpu.gyroZ() - gyroBias[2];
  angle += gyroZ * deltaTime;
}

void driveMotors(double pidOutput) {
  int leftPower = constrain(pidOutput, -200, 200);
  int rightPower = constrain(-pidOutput, -200, 200);

  // Apply deadband compensation
  if (abs(leftPower) < 80) leftPower = (leftPower > 0) ? 80 : -80;
  if (abs(rightPower) < 80) rightPower = (rightPower > 0) ? 80 : -80;

  moveMotor(1, (leftPower > 0) ? 0 : 1, abs(leftPower));  // Left motor
  moveMotor(2, (rightPower > 0) ? 0 : 1, abs(rightPower)); // Right motor
}

void stopMotors(void) {
  moveMotor(1, 1, 0);
  moveMotor(2, 1, 0);
}

void moveMotor(int motorNumber, int direction, int pwmPow) {
  //motorNumber: 1 == left; 2 == right
  // direction == 1 - forward
  // direction == 0 -backward

  if (motorNumber == 1) {
    if (direction == 1) {
      digitalWrite(MOTOR_LEFT_IN1, LOW);
      digitalWrite(MOTOR_LEFT_IN2, HIGH);
    }
    else {
      digitalWrite(MOTOR_LEFT_IN1, HIGH);
      digitalWrite(MOTOR_LEFT_IN2, LOW);
    }
    analogWrite(MOTOR_LEFT_PWM, pwmPow);
  }
  else if (motorNumber == 2) {
    if (direction == 1) {
      digitalWrite(MOTOR_RIGHT_IN3, LOW);
      digitalWrite(MOTOR_RIGHT_IN4, HIGH);
    }
    else {
      digitalWrite(MOTOR_RIGHT_IN3, HIGH);
      digitalWrite(MOTOR_RIGHT_IN4, LOW);
    }
    analogWrite(MOTOR_RIGHT_PWM, pwmPow);
  }
  else{
    Serial.print("Error: motor nonexistent - pausing execution");
    while(1) {
      ;
    }
  }
}

void reset_PID() {
  myPID.SetMode(MANUAL);
  Output = 0;
  myPID.SetMode(AUTOMATIC);
}

void turnLeft() {
  turnRobot(-85.0); // turn 90 degrees left (-90 with offset)
  delay(100);
}

void turnRight() {
  turnRobot(85.0); // turn 90 degrees right (+90 with offset)
  delay(100);
}