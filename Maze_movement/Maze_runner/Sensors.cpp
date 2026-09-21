#include "Sensors.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS, 
  TCS34725_GAIN_4X
);

// Define global variables (only once)
VL53L0X sensorLeft, sensorFront, sensorRight;
double distLeft, distRight, distFront;
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// PID Wall Following
double error;
double correction;
double setpoint = 0.0;
double Kp = 1.0, Ki = 0, Kd = 2.8; // was 0.8  ||  0  ||  2.6
PID wallFollowerPID(&error, &correction, &setpoint, Kp, Ki, Kd, DIRECT);

// Gyro PID
MPU9250_asukiaaa mpu;
double KpGyro = 1.0, KiGyro = 0.01, KdGyro = 0.15;
double angle = 0, SetpointGyro = 0, Input = 0, Output = 0;
unsigned long lastTime = 0;
PID gyroPID(&Input, &Output, &SetpointGyro, KpGyro, KiGyro, KdGyro, DIRECT);
float gyroBias[3] = {0, 0, 0};

void setup_pids(void) {
    wallFollowerPID.SetTunings(Kp, Ki, Kd); // if values are modified later
    gyroPID.SetTunings(KpGyro, KiGyro, KdGyro); // if values are modified later
}

void IRAM_ATTR onLeftEncoder(void) {
    leftTicks += 1;
}
void IRAM_ATTR onRightEncoder(void) {
    rightTicks += 1;
}

void setup_encoders(void) {
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), onLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), onRightEncoder, RISING);
}

void setup_infrareds(void) {
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

// for infrareds, sensorNumber = 1 - left; = 2 - front; = 3 - right
double get_sensor_data_with_filter(int sensorNumber)
{
    double distance, alpha = 0.2; // Smoothing factor (0.2-0.5)

    switch (sensorNumber) {
        case 1: // left sensor
            // distLeft = ;-
            distance = sensorLeft.readRangeContinuousMillimeters();
            //distance = alpha * (sensorLeft.readRangeContinuousMillimeters() - leftOffset) + (1 - alpha) * distLeft;
            break;
        case 2: // front sensor
            distance = sensorFront.readRangeContinuousMillimeters();
            // distFront = sensorFront.readRangeContinuousMillimeters();
            //distance = alpha * (sensorFront.readRangeContinuousMillimeters() - frontOffset) + (1 - alpha) * distFront;
            break;
        case 3: // right sensor
            distance = sensorRight.readRangeContinuousMillimeters();
            // distRight = sensorRight.readRangeContinuousMillimeters();
            //distance = alpha * (sensorRight.readRangeContinuousMillimeters() - frontOffset) + (1 - alpha) * distRight;
            break;
        default:
            distance = 0;
            break;
    }

    return distance;
}

void setup_mpu(void) {
    Wire1.begin(MPU_SDA, MPU_SCL);
    mpu.setWire(&Wire1);
    //mpu.beginAccel();
    mpu.beginGyro();
    lastTime = millis();
}

void calibrate_gyro(void) {
  delay(100); // wait to start calibration until robot is stationary

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
}

float normalize_angle(float angle) {
  angle = fmod(angle, 360); // Reduce to [0, 360)

  if (angle > 180) // normaliza in the [-180, 180] range
    angle -= 360;

  return angle;
}

void update_angle(void) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  mpu.gyroUpdate();
  
  float gyroZ = mpu.gyroZ() - gyroBias[2];
  angle += gyroZ * deltaTime;
}

void reset_gyro_PID(void) {
  gyroPID.SetMode(MANUAL);
  Output = 0;
  gyroPID.SetMode(AUTOMATIC);
}

void reset_wall_PID(void) {
  wallFollowerPID.SetMode(MANUAL);
  correction = 0;
  wallFollowerPID.SetMode(AUTOMATIC);
}

int detectColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  uint8_t nr = map(r, 0, 65535, 0, 255);
  uint8_t ng = map(g, 0, 65535, 0, 255);
  uint8_t nb = map(b, 0, 65535, 0, 255);

  Serial.print("Color R:"); Serial.print(nr);
  Serial.print(" G:"); Serial.print(ng);
  Serial.print(" B:"); Serial.println(nb);

  if (nr < 55 && ng < 55 && nb < 55)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

bool isFinish(void) {
  return detectColor();
}