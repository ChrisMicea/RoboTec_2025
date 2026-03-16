#include "Robot_Movement.h"
#include "Sensors.h"

void move_robot_1_quadrant(void) {
    //reset_wall_PID();

    //noInterrupts();
    long avgTicks = (leftTicks + rightTicks) / 2;
    //interrupts();

    while (avgTicks < target_ticks) {
      // Read distances
      double alpha = 0.3;  // Smoothing factor (0.2-0.5)
      distLeft = sensorLeft.readRangeContinuousMillimeters();  
      distRight = sensorRight.readRangeContinuousMillimeters();

      if (justTurned){
        if (distLeft > MAXVALID)
          distLeft = 50;
        if (distRight > MAXVALID)
          distRight = 50;
      }

      // Compute filtered distances - exponential (?) filtering
      distLeft = alpha * (distLeft - leftOffset) + (1 - alpha) * distLeft;
      distRight = alpha * (distRight - rightOffset) + (1 - alpha) * distRight;

      // Calculate error: positive means too close to left wall
      error = distLeft - distRight; // scale to cm from mm sensor output
      // int err = distLeft - distRight;
      // err *= 0.15; // due tuning

      // Compute PID correction
      wallFollowerPID.Compute();

      // apply correction to motors, while taking deceleration when close to goal in mind
      //noInterrupts(); // for safety
      avgTicks = (leftTicks + rightTicks) / 2;
      //interrupts();
      // int dynamicSpeed = getDeceleratedSpeed(avgTicks, baseSpeed, 60, target_ticks);
      int dynamicSpeed = baseSpeed;
      int PWM_Left = constrain((dynamicSpeed - correction), 60, maxPWM);
      int PWM_Right = constrain((dynamicSpeed + correction), 60, maxPWM);

      // Drive motors
      drive_motors(PWM_Left, PWM_Right);
      delay(10);
    }

    // stop_motors();
    //active_break();
    leftTicks = 0;
    rightTicks = 0;
    delay(10); // for contest - 10
    return;
}

void active_break(void) {
  drive_motors(-90, -90);
  delay(100);
  stop_motors();
  delay(50);
}

void drive_motors(int leftPower, int rightPower) {
    move_motor(1, (leftPower > 0) ? 1 : 0, abs(leftPower));  // Left motor
    move_motor(2, (rightPower > 0) ? 1 : 0, abs(rightPower)); // Right motor
}

void stop_motors(void) {
    move_motor(1, 1, 0);
    move_motor(2, 1, 0);
}

void move_motor(int motorNumber, int direction, int pwmPow) {
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

int getDeceleratedSpeed(long avgTicks, int maxSpeed, int minSpeed, long targetTicks) {
  long remainingTicks = targetTicks - avgTicks;
  const long slowDownRange = 100; // Start slowing down when 100 ticks remain
  
  // stop conditions for deceleration
  if (remainingTicks <= 0)
   return 0;
  if (remainingTicks >= slowDownRange) 
    return maxSpeed;

  // Linear ramp-down
  float ratio = (float)remainingTicks / slowDownRange;
  return minSpeed + ratio * (maxSpeed - minSpeed);
}

void turn_robot(float targetAngle) {
  reset_gyro_PID(); // Reset internal PID state - ensures no carry-over from previous turns
  gyroPID.SetTunings(KpGyro, KiGyro, KdGyro); // not sure if necessary
 
  angle = 0; // Reset angle

  SetpointGyro = normalize_angle(targetAngle);
  
  lastTime = millis(); // initialize lastTime - so it deosn't start with 0, flubbing the calculations form the get-go
  
  unsigned long endTime = millis() + TURN_TIMEOUT; // forcefully stops the movement after 1.2 seconds 
  
  while (abs(normalize_angle(angle - SetpointGyro)) > ANGLE_TOLERANCE && millis() < endTime) { // 
    update_angle();

    Input = normalize_angle(angle);
    gyroPID.Compute();
    
    // int leftPower = constrain(Output, -MAX_TURN_PWM, MAX_TURN_PWM);
    // int rightPower = constrain(-Output, -MAX_TURN_PWM, MAX_TURN_PWM);

    // // Apply deadband compensation
    // if (abs(leftPower) < MIN_TURN_PWM) 
    //   leftPower = (leftPower > 0) ? MIN_TURN_PWM : -MIN_TURN_PWM;
    // if (abs(rightPower) < MIN_TURN_PWM) 
    //   rightPower = (rightPower > 0) ? MIN_TURN_PWM : -MIN_TURN_PWM;

    // Apply minimum power threshold
    if (Output > 0 && Output < MIN_TURN_PWM) Output = MIN_TURN_PWM;
    if (Output < 0 && Output > -MIN_TURN_PWM) Output = -MIN_TURN_PWM;

    drive_in_turns(Output);

    delay(10);
  }
  stop_motors();
}

void turn_left(void) {
  reset_gyro_PID();

  float angle = -85; // turn -90 degrees = 270 degrees with offset

  turn_robot(angle);

  delay(100);
}

void turn_right(void) {
  reset_gyro_PID();

  float angle = 85; // turn 90 degrees with offset

  turn_robot(angle);

  delay(100);
}

void drive_in_turns(double pidOutput) {
  // Increased power range and minimum threshold
  int leftPower = constrain(pidOutput, -MAX_TURN_PWM, MAX_TURN_PWM);
  int rightPower = constrain(-pidOutput, -MAX_TURN_PWM, MAX_TURN_PWM);
  
  // Apply deadband compensation
  if (abs(leftPower) < MIN_TURN_PWM) leftPower = (leftPower > 0) ? MIN_TURN_PWM : -MIN_TURN_PWM;
  if (abs(rightPower) < MIN_TURN_PWM) rightPower = (rightPower > 0) ? MIN_TURN_PWM : -MIN_TURN_PWM;
  
  move_motor(1, (leftPower > 0) ? 0 : 1, abs(leftPower));
  move_motor(2, (rightPower > 0) ? 0 : 1, abs(rightPower));
}