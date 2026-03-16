#include <PID_v1.h>
#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <VL53L0X.h>

// === Constants ===
#define WHEEL_DIAMETER_CM 6
#define ENCODER_TICKS_PER_REV 20
#define QUADRANT_SIZE 100.0

#define LEFT_ENCODER_PIN 21
#define RIGHT_ENCODER_PIN 20
#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_PWM 15
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7
#define MOTOR_RIGHT_PWM 16
// #define INFRA_LEFT_SDA 38
// #define INFRA_LEFT_SCL 39
// #define INFRA_FRONT_SDA 47
// #define INFRA_FRONT_SCL 48
// #define INFRA_RIGHT_SDA 40
// #define INFRA_RIGHT_SCL 41
#define XSHUT_LEFT 41
#define XSHUT_FRONT 42
#define XSHUT_RIGHT 2
#define INFRA_SCL 48
#define INFRA_SDA 47
#define MPU_SDA 8
#define MPU_SCL 9

// infrared sensor declarations
VL53L0X sensorLeft, sensorFront, sensorRight;

const float wheel_circumference = WHEEL_DIAMETER_CM * PI;
const float ticks_per_cm = ENCODER_TICKS_PER_REV / wheel_circumference;
const int target_ticks = QUADRANT_SIZE * ticks_per_cm;
const int maxValid = 40000; // maximum valid distance of sensor output from walls, without assuming empty space

const int baseSpeed = 150;  // Base PWM speed

// === MPU Setup ===
MPU9250_asukiaaa mpu;
float yawAngle = 0;
unsigned long lastUpdateTime = 0;

// === Encoders ===
volatile long leftTicks = 0;
volatile long rightTicks = 0;

void IRAM_ATTR onLeftEncoder() {
  leftTicks += 1;
}
void IRAM_ATTR onRightEncoder() {
  rightTicks += 1;
}

// TwoWire I2C_MPU = TwoWire(0);  // Use I2C bus 0 for MPU
// TwoWire I2C_TOF = TwoWire(1);  // Use I2C bus 1 for Time-of-Flight sensors

void setupEncoders() {
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), onRightEncoder, RISING);
}
void setupMPU() {
  Wire1.begin(MPU_SDA, MPU_SCL); 
  //mpu.setWire(&I2C_MPU);  // Use dedicated I2C bus  
  mpu.setWire(&Wire1);
  mpu.beginAccel();
  mpu.beginGyro();
  // delay(2000);    // Allow sensor to stabilize
}
const int rightOffset = 20; // 20 mm offset
const int leftOffset = 0; // 0 mm offset
const int frontOffset = 0; // 0 mm offset
void setupInfrareds() {
  Wire.begin(INFRA_SDA, INFRA_SCL);

  // Set XSHUT pins as outputs
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  // === Bring up sensor 1 ===
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  sensorLeft.init(true);
  sensorLeft.setAddress(0x30);

  // === Bring up sensor 2 ===
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  sensorFront.init(true);
  sensorFront.setAddress(0x31);

  // === Bring up sensor 3 ===
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  sensorRight.init(true);
  sensorRight.setAddress(0x32);

  sensorLeft.setTimeout(500);  // Set timeout to 500ms - time the sensor waits for input until it times out
  sensorLeft.startContinuous();  // Start continuous mode
  sensorFront.setTimeout(500);  // Set timeout to 500ms - time the sensor waits for input until it times out
  sensorFront.startContinuous();  // Start continuous mode
  sensorRight.setTimeout(500);  // Set timeout to 500ms - time the sensor waits for input until it times out
  sensorRight.startContinuous();  // Start continuous mode
}

// === PID Controllers ===
// Encoder PID
double pidInput, pidOutput, pidSetpoint = 0;
double Kp = 0.8, Ki = 0.01, Kd = 0.3;
PID pidController(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Yaw PID (MPU)
double yawInput, yawOutput, yawSetpoint = 0;
double yawKp = 1.0, yawKi = 0.01, yawKd = 0.15;
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
// Gyro bias offsets (X, Y, Z)
float gyroBias[3] = {0, 0, 0};

// === Wall-following PID ===
unsigned int leftDist, rightDist;
double wallInput, wallOutput, wallSetpoint = 0; // Desired diff = 0
double wallKp = 0.01, wallKi = 0.01, wallKd = 0.3; 
PID wallPID(&wallInput, &wallOutput, &wallSetpoint, wallKp, wallKi, wallKd, DIRECT);

// Add weighting factor for yaw correction
const float YAW_WEIGHT = 0; // Give gyro corrections more influence (was 1.0)
const float ENCODER_WEIGHT = 0; // Reduce encoder influence
const float WALL_WEIGHT = 1; // Weight for wall correction

// === Setup ===
void setup() {
  Serial.begin(115200);

  setupInfrareds();

  // Motor pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  setupEncoders();

  // PID settings
  pidController.SetMode(AUTOMATIC);
  // pidController.SetOutputLimits(-20, 20);
  // pidController.SetSampleTime(20);

  yawPID.SetMode(AUTOMATIC);
  // yawPID.SetOutputLimits(-15, 15);
  // yawPID.SetSampleTime(20);

  wallPID.SetMode(AUTOMATIC);

  setupMPU();
  calibrateGyro();
  mpu.gyroUpdate();
  yawAngle = 0;
  yawSetpoint = 0;
  // delay(500);
  //kickMotors();

  lastUpdateTime = millis();
}

// === Main Loop ===
void loop() {
  // End condition
  long avgTicks = (leftTicks + rightTicks) / 2;
  if (avgTicks >= target_ticks) {
    stopMotors();
    return;
  }

  // === MPU Yaw Update ===
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0;
  lastUpdateTime = now;

  mpu.gyroUpdate();
  float gyroZ = mpu.gyroZ() - gyroBias[2];  // deg/sec
  yawAngle += gyroZ * dt;
  yawInput = yawAngle;
  yawPID.Compute();

  // === Encoder PID Update ===
  pidInput = (double)(leftTicks - rightTicks);
  pidController.Compute();

  leftDist = sensorLeft.readRangeContinuousMillimeters() - leftOffset;
  rightDist = sensorRight.readRangeContinuousMillimeters() - rightOffset;

  Serial.print("left distance is: ");
  Serial.println(leftDist);
  Serial.print("right distance is: ");
  Serial.println(rightDist);
  // === Wall PID Update ===
  wallInput = leftDist - rightDist;
  wallInput *= -WALL_WEIGHT;
  if (leftDist > maxValid || rightDist > maxValid) {
    // wallInput = 0; // Disable wall correction in open space
    stopMotors();
    while(1){
      ;
    }
  }
  wallPID.Compute();

  // === Combine All Three Corrections ===
  // Apply weighted combination
  double totalCorrection = (pidOutput * ENCODER_WEIGHT) + (yawOutput * YAW_WEIGHT + (wallOutput));

  // Add dynamic speed adjustment based on wall error
  int dynamicBaseSpeed = baseSpeed - abs(wallOutput); // Slow down when correcting
  dynamicBaseSpeed = constrain(dynamicBaseSpeed, 100, 255); // Maintain minimum speed
  
  int leftPWM = dynamicBaseSpeed - totalCorrection;
  int rightPWM = dynamicBaseSpeed + totalCorrection;

  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  driveForward(leftPWM, rightPWM);

  // // === Debug Info ===
  // Serial.print("Yaw: "); Serial.print(yawAngle);
  // Serial.print(" | EncErr: "); Serial.print(pidInput);
  // Serial.print(" | Corr: "); Serial.print(totalCorrection);
  // Serial.print(" | PWM L/R: "); Serial.print(leftPWM); Serial.print("/"); Serial.println(rightPWM);
  // Serial.print("Sensor 1: ");
  // Serial.print(sensorLeft.readRangeContinuousMillimeters());
  // Serial.print(" mm | Sensor 2: ");
  // Serial.print(sensorFront.readRangeContinuousMillimeters());
  // Serial.print(" mm | Sensor 3: ");
  // Serial.print(sensorRight.readRangeContinuousMillimeters());
  // Serial.println(" mm");
  delay(100);
}

void kickMotors() {
  int PWM_Pow = 130;
  unsigned long kickTime = millis() + 300;

  while (millis() < kickTime)
    driveForward(PWM_Pow, PWM_Pow);
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

// === Motor Helpers ===
void driveForward(int powLeft, int powRight) {
  moveMotor(1, 1, powLeft);
  moveMotor(2, 1, powRight);
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