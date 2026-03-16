#include <PID_v1.h>

// physical constants
#define WHEEL_DIAMETER_CM 6
#define ENCODER_TICKS_PER_REV 23
#define QUADRANT_SIZE 26.0

// pin defines
#define LEFT_ENCODER_PIN 11
#define RIGHT_ENCODER_PIN 10
#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_PWM 15
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7
#define MOTOR_RIGHT_PWM 16

const float wheel_circumference = WHEEL_DIAMETER_CM * PI;
const float ticks_per_cm = ENCODER_TICKS_PER_REV / wheel_circumference;
const int target_ticks = QUADRANT_SIZE * ticks_per_cm;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

double pidInput, pidOutput, pidSetpoint;
double Kp = 1.5, Ki = 0.05, Kd = 0.1;

PID pidController(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

const int baseSpeed = 150;  // Base motor PWM

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

void setup() {
  // set pin modes
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  pinMode(MOTOR_LEFT_IN1,OUTPUT);
  pinMode(MOTOR_LEFT_IN2,OUTPUT);
  pinMode(MOTOR_LEFT_PWM,OUTPUT);
  pinMode(MOTOR_RIGHT_IN3,OUTPUT);
  pinMode(MOTOR_RIGHT_IN4,OUTPUT);
  pinMode(MOTOR_RIGHT_PWM,OUTPUT);

  setupEncoders();
  pidSetpoint = 0;  // Goal is to keep error (tick difference) = 0
  pidController.SetMode(AUTOMATIC);
  pidController.SetOutputLimits(-50, 50);  // Limit correction range
}

void loop() {
  long avgTicks = (leftTicks + rightTicks) / 2;

  if (avgTicks >= target_ticks) {
    stopMotors();
    return;
  }

  pidInput = (double)(leftTicks - rightTicks);  // Input is the tick error
  pidController.Compute();  // Calculate PID correction

  // Adjust speeds based on output
  int leftPWM = baseSpeed - pidOutput;
  int rightPWM = baseSpeed + pidOutput;

  // Clamp PWM values (optional, if motor driver requires)
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  driveForward(leftPWM, rightPWM);  // Apply corrected motor speeds
}

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
