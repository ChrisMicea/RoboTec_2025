#include <PID_v1.h>
#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <VL53L0X.h>

// === Constants ===
#define WHEEL_DIAMETER_CM 6
#define ENCODER_TICKS_PER_REV 20
#define QUADRANT_SIZE 26.0

// pins
#define LEFT_ENCODER_PIN 21
#define RIGHT_ENCODER_PIN 20
#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_PWM 15
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7
#define MOTOR_RIGHT_PWM 16
#define XSHUT_LEFT 41
#define XSHUT_FRONT 42
#define XSHUT_RIGHT 2
#define INFRA_SCL 48
#define INFRA_SDA 47
#define MPU_SDA 8
#define MPU_SCL 9

VL53L0X sensorLeft, sensorFront, sensorRight;

const int rightOffset = 25; // 25 mm offset
const int leftOffset = 0; // 0 mm offset
const int frontOffset = -10; // -10 mm offset
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
// Global variables for filtering infrared data
#define FILTER_SIZE 5
double distLeftBuffer[FILTER_SIZE], distRightBuffer[FILTER_SIZE];
int filterIndex = 0;

// === Encoders ===
volatile long leftTicks = 0;
volatile long rightTicks = 0;
void IRAM_ATTR onLeftEncoder() {
  leftTicks += 1;
}
void IRAM_ATTR onRightEncoder() {
  rightTicks += 1;
}
void setupEncoders() {
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), onRightEncoder, RISING);
}
const float wheel_circumference = WHEEL_DIAMETER_CM * PI;
const float ticks_per_cm = ENCODER_TICKS_PER_REV / wheel_circumference;
const int target_ticks = QUADRANT_SIZE * ticks_per_cm;


// PID Variables
double error;              // Input to PID
double correction;         // Output from PID
double setpoint = 0.0;     // We want equal distance from both walls

// PID tuning parameters (adjust these empirically)
// double Kp = 0.0005, Ki = 0.05, Kd = 16.0;
// GOOD VARIANT double Kp = 0.8, Ki = 0, Kd = 2.6;
double Kp = 1.0, Ki = 0, Kd = 2.8;
// Start with Ki = 0 and adjust Kp until the robot responds well.
// Add Kd if it's oscillating.
// Add Ki if there's a steady-state error (e.g., robot drifts to one wall over time).

  // Create PID controller
PID wallFollowerPID(&error, &correction, &setpoint, Kp, Ki, Kd, DIRECT);

const int maxValid = 40000; // maximum valid distance of sensor output from walls, without assuming empty space
double distLeft, distRight; // distances measured by sensors

// Constants
const int baseSpeed = 120;  // Base motor speed (adjust as needed)
const int maxPWM = 200;     // Max PWM value

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

  wallFollowerPID.SetMode(AUTOMATIC);
  wallFollowerPID.SetSampleTime(10); // match the loop() delay
  wallFollowerPID.SetOutputLimits(-35, 35); // Limit how much we can steer
}

void loop() {
  long avgTicks = (leftTicks + rightTicks) / 2;

  // if (avgTicks >= target_ticks){
  //   stopMotors();
  //   delay(1000); // for contest - 10
  //   return;
  // }

  moveRobot();
}

void moveRobot() {
  double alpha = 0.3;  // Smoothing factor (0.2-0.5)
  distLeft = sensorLeft.readRangeContinuousMillimeters();  
  distRight = sensorRight.readRangeContinuousMillimeters();
  distLeft = alpha * (distLeft - leftOffset) + (1 - alpha) * distLeft;
  distRight = alpha * (distRight - rightOffset) + (1 - alpha) * distRight;

  // Calculate error: positive means too close to left wall
  error = distLeft - distRight;
  // error = error / 40;

  // Compute PID correction
  wallFollowerPID.Compute();

  // apply correction to motors, while taking deceleration when close to goal in mind
  long avgTicks = (leftTicks + rightTicks) / 2;
  //int dynamicSpeed = getDeceleratedSpeed(avgTicks, baseSpeed, 60, target_ticks);
  int dynamicSpeed = baseSpeed;
  int PWM_Left = constrain(dynamicSpeed - correction, 60, maxPWM);
  int PWM_Right = constrain(dynamicSpeed + correction, 60, maxPWM);


  // Drive motors
  driveMotors(PWM_Left, PWM_Right);
}

// int getDeceleratedSpeed(long avgTicks, int maxSpeed, int minSpeed, long targetTicks) {
//   long remainingTicks = targetTicks - avgTicks;
//   const long slowDownRange = 100; // Start slowing down when 100 ticks remain
//   if (remainingTicks <= 0)
//    return 0;
//   if (remainingTicks >= slowDownRange) 
//     return maxSpeed;

//   // Linear ramp-down
//   float ratio = (float)remainingTicks / slowDownRange;
//   return minSpeed + ratio * (maxSpeed - minSpeed);
// }

void driveMotors(int leftPower, int rightPower) {
  moveMotor(1, (leftPower > 0) ? 1 : 0, abs(leftPower));  // Left motor
  moveMotor(2, (rightPower > 0) ? 1 : 0, abs(rightPower)); // Right motor
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
