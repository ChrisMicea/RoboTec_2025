#include <Wire.h>
#include <FastIMU.h>
#include <PID_v1.h>

#define PERFORM_CALIBRATION
#define MPU_SDA_PIN 8
#define MPU_SCL_PIN 9
#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_PWM 15
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7
#define MOTOR_RIGHT_PWM 16

MPU6500 mpu;

calData calib = { 0 };
GyroData gyroData;
double angle = 0, Setpoint, Input, Output;
unsigned long lastTime = 0;

// PID Setup
double Kp = 1.5, Ki = 0, Kd = 0.3;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(MOTOR_LEFT_IN1,OUTPUT);
  pinMode(MOTOR_LEFT_IN2,OUTPUT);
  pinMode(MOTOR_LEFT_PWM,OUTPUT);
  pinMode(MOTOR_RIGHT_IN3,OUTPUT);
  pinMode(MOTOR_RIGHT_IN4,OUTPUT);
  pinMode(MOTOR_RIGHT_PWM,OUTPUT);

  Serial.begin(115200);
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  //mpu.initialize();
  // mpu.begin();
  int err = mpu.init(calib);
  if (err != 0) {
    Serial.println("Error initializing MPU6500");
    while (1) {
      ;
    }
  }
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  // calibrateGyro();

  #ifdef PERFORM_CALIBRATION
    Serial.println("FastIMU calibration & data example");
    delay(2500);
    Serial.println("Keep mpu level.");
    delay(5000);
    mpu.calibrateAccelGyro(&calib);
    Serial.println("Calibration done!");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calib.accelBias[0]);
    Serial.print(", ");
    Serial.print(calib.accelBias[1]);
    Serial.print(", ");
    Serial.println(calib.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calib.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calib.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calib.gyroBias[2]);
    delay(5000);
    mpu.init(calib);
  #endif  
}

void loop() {
  turnRobot(90.0); // Turn 90 degrees
  delay(2000);
}

float normalizeAngle(float angle) {
  angle = fmod(angle, 360); // Reduce to [0, 360)
  if (angle > 180) angle -= 360;
  return angle;
}

void updateAngle() {
  // unsigned long currentTime = millis();
  // float deltaTime = (currentTime - lastTime) / 1000.0;
  // lastTime = currentTime;
  mpu.getGyro(&gyroData);
  float gyroZ = gyroData.gyroZ - calib.gyroBias[2];
  // angle += gyroZ * deltaTime;
  Serial.print("GyroZ: "); Serial.print(gyroZ);
  Serial.print(" | Angle: "); Serial.println(angle);
}

void turnRobot(float targetAngle) {
  angle = 0; // Reset angle
  Setpoint = normalizeAngle(targetAngle);
  while (abs(normalizeAngle(angle - Setpoint)) > 2.0) {
    updateAngle();
    // Input = normalizeAngle(angle);
    // myPID.Compute();
    // driveMotors(Output);
    delay(10);
  }
  stopMotors();
}

void driveMotors(double pidOutput) {
  int leftPower = constrain(pidOutput, -255, 255);
  int rightPower = constrain(-pidOutput, -255, 255);
  // analogWrite(MOTOR_LEFT_PWM, abs(leftPower));
  moveMotor(1, (leftPower > 0) ? 1 : 0, abs(leftPower));  // Left motor
  moveMotor(2, (rightPower > 0) ? 1 : 0, abs(rightPower)); // Right motor
  // analogWrite(MOTOR_RIGHT_PWM, abs(rightPower));
  // Set direction pins if needed
}

// void driveForward(int powLeft, int powRight) {
//   moveMotor(1, 1, powLeft);
//   moveMotor(2, 1, powRight);
// }
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
