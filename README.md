# RoboTec 2025 - Maze Solving Robot

## Overview
This codebase is for an autonomous maze-solving robot built for the **RoboTec 2025 competition** (Maze track). The robot uses depth-first search (DFS) to explore and navigate through a maze, detecting walls with infrared sensors and using gyroscope-assisted precise turning.

## ⚠️ Important Notice
**This is an embedded application and will not be runnable without our specific hardware setup.** The code is designed for our custom robot configuration and requires the exact hardware components listed below.

## Hardware Specifications

### Power System
- **Custom 12V Rechargeable Battery**: Built by connecting three 3.7V lithium-ion batteries in series using spot welding
- Provides power to motors and all onboard electronics

### Sensors
- **VL53L0X Time-of-Flight Sensors** (3x): Left, Front, Right - for wall detection
- **MPU9250 Gyroscope**: For precise angular measurement during turns
- **Wheel Encoders** (2x): Optical encoders for odometry and distance tracking
- **Adafruit TCS34725 Color Sensor**: For finish line detection (black color)

### Motors
- **DC Motors** (2x): With H-bridge driver for differential drive
- PWM speed control with encoder feedback

### Microcontroller
- ESP32-based board (compatible with Arduino framework)

## Team
Built through the collaboration of:
- Christian Micea - me
- [Andrei Orbulescu](https://github.com/Andrei2x32)
- [Dragoș Mâțu](https://github.com/Saltspeddy)
- [Cosmin Murariu](https://github.com/CosminM12)

## Development Photos
We do not have a clear image of the finished robot, but these photos show two stages in its development:

### Custom Battery Assembly
![Custom Battery](RoboTec_2025/images/custom_battery.jpeg)
*Our custom 12V rechargeable battery built by connecting three 3.7V lithium-ion batteries in series using spot welding*

### Rough Component Assembly
![Component Assembly](RoboTec_2025/images/rough_component_outline.jpeg)
*Early stage assembly showing sensor placement and component layout*

## Project Structure
```
RoboTec_2025/
├── Maze_movement/
│   ├── Maze_runner/          # Main maze-solving implementation
│   │   ├── main.ino          # DFS navigation algorithm
│   │   ├── Robot_Movement.h/cpp  # Motor control & movement
│   │   └── Sensors.h/cpp     # Sensor processing & PID control
│   └── Basic_outline/        # Earlier version/outline
├── IMU_tests/                # Individual component tests
│   ├── gyro_turn/            # Gyroscope turning tests
│   ├── encoder_test/         # Encoder validation
│   └── wall_following_v3/    # Wall following PID tests
├── basics_tests/             # Basic hardware tests
│   ├── motors_PWM/           # Motor control tests
│   ├── infrared_test/        # IR sensor tests
│   └── ultrasonic_test/      # Ultrasonic sensor tests
└── libraries/                # External dependencies (gitignored)
```

## Dependencies

### Required Arduino Libraries
Install these via Arduino Library Manager or from the provided links:

- **Adafruit TCS34725** - [Library Manager](https://www.adafruit.com/product/1334)
- **VL53L0X** by Pololu - [Library Manager](https://www.pololu.com/product/2490)
- **PID_v1** by Brett Beauregard - [Library Manager](https://github.com/br3ttb/Arduino-PID-Library)
- **MPU9250_asukiaaa** - [GitHub](https://github.com/asukiaaa/MPU9250_asukiaaa)
- **Adafruit BusIO** - [Library Manager](https://github.com/adafruit/Adafruit_BusIO)

### Built-in Libraries
- **Wire.h** - I2C communication
- **BluetoothSerial** - ESP32 built-in (for OTA/debugging)

## Key Features

### Navigation Algorithm
- **Depth-First Search (DFS)**: Explores maze recursively, marking visited cells
- **Coordinate Tracking**: Maintains position (X, Y) and heading (N, E, S, W)
- **Single-pass exploration** (backtracking not fully implemented)

### Movement Control
- **Encoder-based Odometry**: Precise quadrant (30cm) movement
- **PID Wall Following**: Maintains center position in corridors
- **Gyroscope-assisted Turning**: Precise 90° turns with PID control
- **Active Braking**: Quick stops using reverse motor power

### Sensor Fusion
- **Infrared Distance Sensors**: Wall detection with 260mm range
- **Gyroscope Calibration**: Startup bias compensation for drift
- **Color Detection**: Finish line recognition (black color threshold)

## Pin Configuration

| Component | Pins |
|-----------|------|
| Left Motor | IN1=4, IN2=5, PWM=15 |
| Right Motor | IN3=6, IN4=7, PWM=16 |
| Left Encoder | 21 |
| Right Encoder | 20 |
| IR Sensors I2C | SDA=48, SCL=47 |
| Gyro I2C | SDA=8, SCL=9 |
| IR XSHUT | Left=41, Front=42, Right=2 |

## Constants
- Wheel diameter: 6cm
- Encoder ticks per revolution: 20
- Maze quadrant size: 30cm
- Base motor speed: 120 PWM
- Max motor speed: 160 PWM
- Wall detection threshold: 100mm

## PID Tuning
- **Wall Following**: Kp=1.0, Ki=0, Kd=2.8
- **Gyroscope Turning**: Kp=1.0, Ki=0.01, Kd=0.15

## Usage
1. Install required libraries (see Dependencies section)
2. Upload code to ESP32 board
3. Place robot at maze start position (0, 0)
4. Power on - robot will automatically begin exploration
5. Robot stops when color sensor detects finish line (black)

## Known Issues
- Backtracking in DFS algorithm incomplete (`oppositeDirection()` not implemented)
- Deceleration ramp code exists but is commented out
- Some sensor filtering code is commented out