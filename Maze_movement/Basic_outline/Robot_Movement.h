#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <PID_v1.h>
#include <mat.h>
#include <Arduino.h>

// === Constants ===
#define WHEEL_DIAMETER_CM 6
#define ENCODER_TICKS_PER_REV 20
#define QUADRANT_SIZE 20.0 // cm
#define MAX_TURN_PWM 200
#define MIN_TURN_PWM 80
#define ANGLE_TOLERANCE 2.0 // degrees
#define TURN_TIMEOUT 1500 // ms

// pin defines
#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_PWM 15
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7
#define MOTOR_RIGHT_PWM 16

constexpr float wheel_circumference = WHEEL_DIAMETER_CM * PI;
constexpr float ticks_per_cm = ENCODER_TICKS_PER_REV / wheel_circumference;
constexpr int target_ticks = QUADRANT_SIZE * ticks_per_cm;

extern bool justTurned;

const int baseSpeed = 80;  // Base PWM speed
const int maxPWM = 110;     // Max PWM value

void stop_motors(void);
void move_motor(int motorNumber, int direction, int pwmPow);
void drive_motors(int leftPower, int rightPower);
void move_robot_1_quadrant(void);
int getDeceleratedSpeed(long avgTicks, int maxSpeed, int minSpeed, long targetTicks);
void active_break(void);

//wrapper function for turn_robot - no need to specify angles
void turn_left(void); 
void turn_right(void);
void turn_robot(float targetAngle);
void drive_in_turns(double pidOutput);

#endif