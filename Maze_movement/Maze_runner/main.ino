#include <map>
#include "Sensors.h"
#include "Robot_Movement.h"

const int WALL_THRESHOLD = 100; // mm

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
Direction heading = NORTH;

int posX = 0, posY = 0; // Robot-relative coordinates
std::map<std::pair<int, int>, bool> visited;

// Function prototypes
void explore(int x, int y);
bool isWallFront();
bool isWallLeft();
bool isWallRight();
void turnTo(Direction target);
void updatePosition();
void move(Direction dir);
bool isBlocked(Direction dir);

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

  lastTime = millis();

  explore(posX, posY);
}

void loop() {
  // Nothing here, we do all logic in setup()
}

// Core recursive DFS logic
void explore(int x, int y) {
  if (isFinish() && (x != 0 || y != 0)) {
    stop_motors();
    while(1); // Stop forever
  }

  visited[{x, y}] = true;

  const Direction directions[] = { NORTH, EAST, SOUTH, WEST };
  for (int i = 0; i < 4; i++) {
    Direction dir = directions[i];
    int nx = x, ny = y;

    switch (dir) {
      case NORTH: 
        ny++; 
        break;
      case EAST:  
        nx++; 
        break;
      case SOUTH: 
        ny--; 
        break;
      case WEST:  
        nx--; 
        break;
    }

    if (visited[{nx, ny}]) 
        continue;
    if (isBlocked(dir)) 
        continue;

    move(dir);
    explore(nx, ny);
    // Optional backtracking: Uncomment if allowed
    move(oppositeDirection(dir));  ///// implement!!!
  }
}

// Wall detection helpers
bool isWallFront()  { 
  //distanceFront()
  return get_sensor_data_with_filter(2) < WALL_THRESHOLD; 
}
bool isWallLeft()   { 
    //distanceLeft()
    return get_sensor_data_with_filter(1) < WALL_THRESHOLD; 
}
bool isWallRight()  { 
    //distanceRight()
    return get_sensor_data_with_filter(3) < WALL_THRESHOLD; 
}

// Block detection based on direction
bool isBlocked(Direction dir) {
  int relative = (dir - heading + 4) % 4;
  switch (relative) {
    case 0: 
        return isWallFront();
    case 1: 
        return isWallRight();
    case 2: 
        return true; // Disallow reverse (single pass)
    case 3: 
        return isWallLeft();
  }
  return true;
}

// Turns robot to face given direction
void turnTo(Direction target) {
  int diff = (target - heading + 4) % 4;
  if (diff == 1) {
    active_break();
    turn_right();
    delay(400);
  } 
  else if (diff == 3) {
    active_break();
    turn_left();
    delay(400);
  } 
  else if (diff == 2) { // U-turn
    active_break();
    turn_right(); 
    delay(200);
    turn_right();
    delay(400);
  }

  heading = target;
}

// Move in a direction and update position
void move(Direction dir) {
  turnTo(dir);
  move_robot_1_quadrant();
  updatePosition();
}

// Adjust position based on heading
void updatePosition() {
  switch (heading) {
    case NORTH: 
        posY++; 
        break;
    case EAST:  
        posX++; 
        break;
    case SOUTH: 
        posY--; 
        break;
    case WEST:  
        posX--; 
        break;
  }
}
