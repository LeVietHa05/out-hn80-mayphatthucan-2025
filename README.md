# XY Servo Catcher System

This project implements an automated XY positioning system with servo-controlled catchers and load cell verification for precise item dispensing and catching.

## Files Overview
- `src/main.cpp`: Main application code for the full XY servo catcher system.
- `test_stepper.ino`: Standalone stepper motor test program for debugging.

## Main Application Code Flow (main.cpp)

### Setup Function
1. Initialize serial communication at 115200 baud.
2. Print startup message.
3. Initialize I2C for servo drivers.
4. Call setup functions for servos, steppers, and load cell.
5. Set initial positions (x=0, y=0).
6. Print system ready message and status.

### Loop Function
1. Check for incoming serial commands and parse them.
2. Run stepper motors (non-blocking) to handle ongoing movements.
3. Handle sequence execution if a sequence is running.
4. Check if movement is complete:
   - Update current position to target.
   - If not in sequence mode, activate the corresponding servo for the position.
   - Print updated status.

### Key Functions

#### Movement and Positioning
- `moveToPosition(float x, float y)`: Converts mm coordinates to steps, sets stepper targets, initiates movement.
- `isAtTarget()`: Checks if both steppers have reached their targets.
- `getServoForPosition(float x, float y)`: Finds the closest servo based on predefined drop points.

#### Servo Control
- `setupServos()`: Initializes two PCA9685 boards (0x40 and 0x41) for 18 servos total, sets PWM frequency, closes all servos.
- `activateServo(int servoIndex, float targetWeight)`: Opens above and below servos sequentially, checks load cell for item detection with retries.

#### Load Cell
- `setupLoadCell()`: Initializes HX711, sets calibration factor, tares the scale.
- `readLoadCell()`: Takes multiple readings and averages for accuracy.
- `isWeightSufficient(float weight, float targetWeight)`: Checks if measured weight meets target within tolerance.

#### Serial Command Parsing
- `parseSerialCommand()`: Handles input buffer, supports:
  - "X,Y" format for direct positioning.
  - "sequence" or "start" to begin predefined sequence.
  - "1,300;5,100;9,200;" format for custom sequences.
- `parseSequenceCommand(String command)`: Parses semicolon-separated drop points with target weights.

#### Sequence Handling
- `startSequence()`: Initiates sequence execution, moves to first point.
- `handleSequence()`: Manages sequence progression, activates servos at each point, moves to next or completes.

### System Configuration
- Stepper motors: 200 steps/rev, 16 microstepping, Tr10x8 lead screw (400 steps/mm).
- Travel limits: X=545mm, Y=675mm.
- 9 drop points in 3x3 grid, each with dedicated servo pair.
- Load cell with configurable calibration and tolerance.

## Stepper Motor Test Program (test_stepper.ino)

This Arduino sketch provides a simple way to test stepper motors using serial commands. It allows manual control of X and Y axes stepper motors for debugging and calibration purposes.

### Hardware Requirements
- Arduino board (e.g., ESP32)
- Two stepper motors with drivers (e.g., A4988 or similar)
- Connections:
  - X-axis Stepper: STEP_PIN_X (19), DIR_PIN_X (18), ENABLE_PIN_X (17)
  - Y-axis Stepper: STEP_PIN_Y (23), DIR_PIN_Y (25), ENABLE_PIN_Y (16)

### Test Code Flow
- **Setup**: Initialize serial, configure steppers with speed/accel, enable outputs, print commands.
- **Loop**: Read serial commands, parse them, run steppers.
- **Commands**: X<steps>, Y<steps>, H (home), S (stop), P (print positions).

## Dependencies
- AccelStepper library
- Adafruit_PWMServoDriver library (for main.cpp)
- HX711 library (for main.cpp)

## Usage
1. For testing: Upload `test_stepper.ino`, use Serial Monitor with commands like "X100".
2. For full system: Upload `src/main.cpp`, send coordinates or sequences via serial.

## Notes
- Ensure proper power supplies for all components.
- Calibrate load cell factor for accurate weight measurements.
- Adjust stepper parameters based on your hardware setup.
