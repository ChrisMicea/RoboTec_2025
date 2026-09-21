#ifndef SENSORS_H
#define SENSORS_H

#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include <PID_v1.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include <Adafruit_TCS34725.h>

// pin defines
#define LEFT_ENCODER_PIN 21
#define RIGHT_ENCODER_PIN 20
#define XSHUT_LEFT 41
#define XSHUT_FRONT 42
#define XSHUT_RIGHT 2
#define INFRA_SCL 48
#define INFRA_SDA 47
#define MPU_SDA 8
#define MPU_SCL 9

#define MAXVALID 320 // maximum valid distance of sensor output from walls, without assuming empty space
// 260 mm = 26 cm = 1 quadrant

// TwoWire I2CBus;

extern VL53L0X sensorLeft, sensorFront, sensorRight;

// offsets for infrared sensors
const int rightOffset = 25; // 25 mm offset
const int leftOffset = 0; // 0 mm offset
const int frontOffset = -10; // 10 mm offset

extern double distLeft, distRight, distFront; // distances measured by sensors

// === Encoders ===
extern volatile long leftTicks;
extern volatile long rightTicks;
void IRAM_ATTR onLeftEncoder(void);
void IRAM_ATTR onRightEncoder(void);
void setup_encoders(void); 

// infrared sensors
void setup_infrareds(void);


// PID Variables for wall following
extern double error;              // Input to PID
extern double correction;         // Output from PID
extern double setpoint;     // We want equal distance from both walls
// Create PID controller
// extern double Kp = 1.0, Ki = 0, Kd = 2.6; // due modification
extern double Kp, Ki, Kd; // due modification
// extern PID wallFollowerPID(&error, &correction, &setpoint, Kp, Ki, Kd, DIRECT);
extern PID wallFollowerPID;

// gyroscope setup
extern MPU9250_asukiaaa mpu;
// extern double KpGyro = 1.0, KiGyro = 0.01, KdGyro = 0.15;
extern double KpGyro, KiGyro, KdGyro;
extern double angle, SetpointGyro, Input, Output;
extern unsigned long lastTime;
// PID for turns
// extern PID gyroPID(&Input, &Output, &SetpointGyro, KpGyro, KiGyro, KdGyro, DIRECT);
extern PID gyroPID;
// Gyro bias offsets (X, Y, Z)
// extern float gyroBias[3] = {0, 0, 0};
extern float gyroBias[3];


// for infrared
double get_sensor_data_with_filter(int sensorNumber);

// for gyroscope
float normalize_angle(float angle);
void update_angle(void);
void calibrate_gyro(void);
// void setup_mpu(void);
void setup_pids(void);
void reset_gyro_PID(void);
void reset_wall_PID(void);
// int detectColor(void);

#endif