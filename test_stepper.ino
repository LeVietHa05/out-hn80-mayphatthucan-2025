#include <AccelStepper.h>

// ===== Pin Definitions =====
#define STEP_PIN_X 19
#define DIR_PIN_X 18
#define ENABLE_PIN_X 17
#define STEP_PIN_Y 23
#define DIR_PIN_Y 25
#define ENABLE_PIN_Y 16

// ===== Stepper Motors =====
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

void setup() {
  Serial.begin(115200);
  Serial.println("Stepper Test System Starting...");

  // Configure stepper parameters
  stepperX.setMaxSpeed(1000);    // steps per second
  stepperX.setAcceleration(500); // steps per second²
  stepperX.setEnablePin(ENABLE_PIN_X);
  stepperX.enableOutputs();

  stepperY.setMaxSpeed(1000);
  stepperY.setAcceleration(500);
  stepperY.setEnablePin(ENABLE_PIN_Y);
  stepperY.enableOutputs();

  Serial.println("Stepper motors ready!");
  Serial.println("Commands:");
  Serial.println("  X<steps>  - Move X axis (e.g., X100, X-50)");
  Serial.println("  Y<steps>  - Move Y axis (e.g., Y200, Y-100)");
  Serial.println("  H         - Home (move to 0,0)");
  Serial.println("  S         - Stop all movement");
  Serial.println("  P         - Print current positions");
}

void loop() {
  // Handle serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    parseCommand(command);
  }

  // Run steppers
  stepperX.run();
  stepperY.run();
}

void parseCommand(String command) {
  if (command.startsWith("X")) {
    // Move X axis
    String stepsStr = command.substring(1);
    long steps = stepsStr.toInt();
    stepperX.move(steps);
    Serial.printf("Moving X by %ld steps\n", steps);
  }
  else if (command.startsWith("Y")) {
    // Move Y axis
    String stepsStr = command.substring(1);
    long steps = stepsStr.toInt();
    stepperY.move(steps);
    Serial.printf("Moving Y by %ld steps\n", steps);
  }
  else if (command.equalsIgnoreCase("H")) {
    // Home - move to 0,0
    stepperX.moveTo(0);
    stepperY.moveTo(0);
    Serial.println("Homing to (0,0)");
  }
  else if (command.equalsIgnoreCase("S")) {
    // Stop all movement
    stepperX.stop();
    stepperY.stop();
    Serial.println("Stopping all movement");
  }
  else if (command.equalsIgnoreCase("P")) {
    // Print positions
    Serial.printf("X: %ld, Y: %ld\n", stepperX.currentPosition(), stepperY.currentPosition());
  }
  else {
    Serial.println("Unknown command. Use X<steps>, Y<steps>, H, S, or P");
  }
}
