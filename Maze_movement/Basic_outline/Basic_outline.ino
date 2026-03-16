#include "Sensors.h"
#include "Robot_Movement.h"

bool atStart = true;

TwoWire I2CBus = TwoWire(1);  // use 0 or 1 depending on ESP32 hardware

const int ledPin = 35;
bool justTurned = false;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS, 
  TCS34725_GAIN_4X
);

void setup_mpu(void) {
    I2CBus.begin(MPU_SDA, MPU_SCL);

    //mpu address - 0x68
    mpu.setWire(&I2CBus);
    //mpu.beginAccel();
    mpu.beginGyro();
    lastTime = millis();
    //byte mpuSetupResult = mpu.begin(MPU9250_asukiaaa::MPU9250_ADDRESS_AD0_LOW, &Wire);

    //TCS address - 0x29
    while (!tcs.begin(TCS34725_ADDRESS, &I2CBus)){
      Serial.println("No TCS34725 found!");
    }
    
}


void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  setup_infrareds();

  // Encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  setup_encoders();

  setup_pids();

  setup_mpu();
  wallFollowerPID.SetMode(AUTOMATIC);
  wallFollowerPID.SetSampleTime(10); // match the loop() delay
  wallFollowerPID.SetOutputLimits(-35, 35); // Limit how much we can steer
  gyroPID.SetMode(AUTOMATIC);
  gyroPID.SetOutputLimits(-255, 255);

  calibrate_gyro();

  pinMode(ledPin, OUTPUT);
  analogWrite(ledPin, 128);

  //scanI2C(I2CBus, "Main Bus (MPU/Color)");

  lastTime = millis();
}

void scanI2C(TwoWire &wire, const char* busName) {
  Serial.print("Scanning "); Serial.print(busName); Serial.println("...");
  byte count = 0;
  
  for (byte i = 8; i < 120; i++) {
    wire.beginTransmission(i);
    if (wire.endTransmission() == 0) {
      Serial.print("Found device at 0x"); Serial.println(i, HEX);
      count++;
    }
    delay(10);
  }
  Serial.print(count); Serial.println(" devices found");
}

bool dirSwitch = false;

void loop() {
  int isBlack = detectColor();
  Serial.print("Color : ");
  Serial.println(isBlack);

  // if(isBlack){ //black
  //   if(atStart)
  //     atStart = !atStart;
  //   else{
  //     Serial.println("black detected");
  //     // stop_motors();
  //     // return;
  //     while(1){
  //       stop_motors();;
  //     }
  //   }
  // }
  
  distLeft = sensorLeft.readRangeContinuousMillimeters();
  distRight = sensorRight.readRangeContinuousMillimeters();
  distFront = sensorFront.readRangeContinuousMillimeters();

  // if (distFront > 160) { // go Froward if able
  //   //move_forward_1_quadrant();
  //   move_robot_1_quadrant();
  //   justTurned = false;
  // }
  // else{
  //   active_break();

  //   if (distRight > MAXVALID) {
  //     turn_right();
  //     delay(500);
  //     justTurned = true;
  //   }
  //   else if(distLeft > MAXVALID) {
  //     turn_left();
  //     delay(500);
  //     justTurned = true;
  //   }
  //   else{
  //     // no path available => U turn
  //     moveBack();
  //     turn_left();
  //     delay(500);
  //     turn_left();
  //     delay(500);
  //     justTurned = true;
  //   }
  // }

  // delay(10);

  if (distLeft > MAXVALID && distRight > MAXVALID) {
    if (justTurned) {
      justTurned = ! justTurned;
    }
    else{
    active_break();
    if (dirSwitch)
      turn_right();
    else
      turn_left();
    dirSwitch = !dirSwitch;
    delay(500);  
    }
  }

  if (distLeft > MAXVALID) {
    if (justTurned) {
      justTurned = ! justTurned;
    }
    else{
      active_break();
      turn_left();
      delay(500);
    }
    }
  
  //test_correction_direction();

  if (distRight > MAXVALID) {
    if (justTurned) {
      justTurned = ! justTurned;
    }
    else{
    active_break();
    turn_right();
    delay(500);
    }
  }

  move_robot_1_quadrant();
  delay(10);
}

int detectColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // uint8_t nr = map(r, 0, 65535, 0, 255);
  // uint8_t ng = map(g, 0, 65535, 0, 255);
  // uint8_t nb = map(b, 0, 65535, 0, 255);

  Serial.print("Color R:"); Serial.print(r);
  Serial.print(" G:"); Serial.print(g);
  Serial.print(" B:"); Serial.println(b);

  if (r < 25 && g < 25 && b < 25)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void moveBack(){
    drive_motors(-90, -90);
    delay(300);
}

//comentat de andrei
/*

void test_correction_direction() {
  // Move close to left wall manually, then:
  error = 100; // Simulate being too close to left wall
  wallFollowerPID.Compute();
  
  // The right motor should get MORE power, left motor LESS
  Serial.print("Correction direction test: ");
  Serial.print("Correction="); Serial.print(correction);
  Serial.print(" => Left PWM="); Serial.print(baseSpeed);
  Serial.print(" Right PWM="); Serial.println(baseSpeed);
  
  if (correction > 0) {
    Serial.println("CORRECT: When too close to left, right motor speeds up");
  } else {
    Serial.println("ERROR: Wrong correction direction!");
    while(1); // Halt if wrong
  }
}
*/