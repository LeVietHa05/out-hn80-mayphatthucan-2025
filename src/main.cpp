#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <HX711.h>
#include <WiFiManager.h>

// ===== Pin Definitions =====
#define STEP_PIN_X 19
#define DIR_PIN_X 18
#define STEP_PIN_Y 23
#define DIR_PIN_Y 25
#define ENABLE_PIN_X 17
#define ENABLE_PIN_Y 16

// ===== HX711 Load Cell =====
#define HX711_DT_PIN 32  // Data pin
#define HX711_SCK_PIN 33 // Clock pin

// ===== System Configuration =====
#define STEPS_PER_ROUND 200
#define MICROSTEP 16 // MS2, MS1: 00: 1/8, 01: 1/32, 10: 1/64 11: 1/16
// Lead screw: Tr10x8 (8mm lead per revolution)
#define LEADSCREW 8
// Stepper: 200 steps/rev with 16 microstepping = 3200 steps/rev
// Steps per mm: 3200 / 8 = 400 steps per mm
#define STEPS_PER_MM_X STEPS_PER_ROUND *MICROSTEP / LEADSCREW // Tr10x8 lead screw with 16 microstepping
#define STEPS_PER_MM_Y STEPS_PER_ROUND *MICROSTEP / LEADSCREW // Tr10x8 lead screw with 16 microstepping
#define MAX_X_MM 545                                          // Maximum X travel in mm
#define MAX_Y_MM 675                                          // Maximum Y travel in mm
#define SERVO_MIN 150                                         // 0° position
#define SERVO_MAX 600                                         // 180° position

// ===== Load Cell Configuration =====
#define LOAD_CELL_CALIBRATION_FACTOR 1000.0 // Calibration factor (adjust based on your load cell)
#define WEIGHT_TOLERANCE 5.0                // Acceptable weight tolerance in grams
#define MAX_RETRIES 3                       // Maximum number of retry attempts
#define LOAD_CELL_READINGS 10               // Number of readings to average

// ===== Drop Point Sequence Configuration =====
#define NUM_DROP_POINTS 3 // Number of drop points to visit

// ===== Servo (PCA9685) =====
// Two PCA9685 boards: one for above servos (0x40), one for below servos (0x41)
Adafruit_PWMServoDriver pwmAbove = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmBelow = Adafruit_PWMServoDriver(0x41);
// 18 servos total: 9 above (channels 0-8) and 9 below (channels 0-8)
int servoChannelsAbove[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
int servoChannelsBelow[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

// ===== Stepper Motors =====
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);

// ===== HX711 Load Cell =====
HX711 loadCell;

// ===== System State =====
struct Position
{
  float x = 0.0;
  float y = 0.0;
  bool isMoving = false;
};

Position currentPos;
Position targetPos;

// ===== Sequence State =====
int currentSequenceIndex = 0;
bool sequenceRunning = false;

// ===== Position to Servo Mapping =====
// Define 9 positions in a 3x3 grid (adjust coordinates based on your setup)
struct DropPoint
{
  float x;
  float y;
  int servoIndex;
};

DropPoint origin = {0, 325, 0};

// ===== Sequence Drop Points =====
// Define drop points to visit with their target weights
struct SequencePoint
{
  float x;
  float y;
  int servoIndex;
  float targetWeight;
};

// Dynamic sequence array
SequencePoint sequencePoints[9]; // Maximum 9 points
int numSequencePoints = 0;

// TODO: test position
DropPoint dropPoints[9] = {
    {536, 505, 0}, // Position 1 -> Servo 0
    {536, 325, 1}, // Position 2 -> Servo 1
    {536, 145, 2}, // Position 3 -> Servo 2
    {356, 505, 3}, // Position 4 -> Servo 3
    {356, 325, 4}, // Position 5 -> Servo 4
    {356, 145, 5}, // Position 6 -> Servo 5
    {176, 505, 6}, // Position 7 -> Servo 6
    {176, 325, 7}, // Position 8 -> Servo 7
    {176, 145, 8}  // Position 9 -> Servo 8
};

// ===== Function Declarations =====
void setupServos();
void setupSteppers();
void setupLoadCell();
void moveToPosition(float x, float y);
bool isAtTarget();
int getServoForPosition(float x, float y);
void activateServo(int servoIndex, float targetWeight);
void parseSerialCommand();
void parseSequenceCommand(String command);
void printStatus();
float readLoadCell();
bool isWeightSufficient(float weight, float targetWeight);
void startSequence();
void handleSequence();

void setup()
{
  Serial.begin(115200);
  Serial.println("=== XY Servo Catcher System Starting ===");

  // WiFiManager setup
  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3 minutes to configure
  if (!wm.autoConnect("KHKT FOOD CATCHER"))
  {
    Serial.println("Failed to connect to WiFi, continuing without WiFi...");
  }
  else
  {
    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  // Initialize I2C for servo driver
  Wire.begin();

  // Setup components
  setupServos();
  setupSteppers();
  setupLoadCell();

  // Set initial position
  currentPos.x = 0;
  currentPos.y = 0;
  targetPos.x = 0;
  targetPos.y = 0;

  Serial.println("System ready! Send coordinates as 'X,Y' (e.g., '100,50')");
  printStatus();
}

void loop()
{
  // WiFi auto reconnect
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    WiFi.reconnect();
    delay(5000); // Wait 5 seconds before checking again
  }

  // Handle serial communication
  if (Serial.available() > 0)
  {
    parseSerialCommand();
  }

  // Run steppers (non-blocking)
  stepperX.run();
  stepperY.run();

  // Handle sequence if running
  handleSequence();

  // Check if movement is complete
  if (currentPos.isMoving && isAtTarget())
  {
    currentPos.isMoving = false;
    currentPos.x = targetPos.x;
    currentPos.y = targetPos.y;

    // Only activate servo if not running a sequence (sequence handles its own activation)
    if (!sequenceRunning)
    {
      // Activate the corresponding servo for single position moves
      int servoIndex = getServoForPosition(currentPos.x, currentPos.y);
      if (servoIndex >= 0)
      {
        // Use default target weight for single moves
        float targetWeight = 100.0;
        activateServo(servoIndex, targetWeight);
        Serial.printf("Arrived at (%.1f, %.1f) - Activated servo %d\n",
                      currentPos.x, currentPos.y, servoIndex);
      }
    }

    printStatus();
  }
}

void setupServos()
{
  Serial.println("Setting up servo system...");

  // Initialize above servos (PCA9685 at 0x40)
  pwmAbove.begin();
  pwmAbove.setPWMFreq(50); // 50Hz for servo motors

  // Initialize below servos (PCA9685 at 0x41)
  pwmBelow.begin();
  pwmBelow.setPWMFreq(50); // 50Hz for servo motors

  // Initialize all 18 servos to closed position (0°)
  for (int i = 0; i < 9; i++)
  {
    pwmAbove.setPWM(servoChannelsAbove[i], 0, SERVO_MIN);
    pwmBelow.setPWM(servoChannelsBelow[i], 0, SERVO_MIN);
  }
  Serial.println("Servo system ready");
}

void setupSteppers()
{
  Serial.println("Setting up stepper motors...");

  // Configure stepper parameters
  stepperX.setMaxSpeed(1000);    // steps per second
  stepperX.setAcceleration(500); // steps per second²
  stepperX.setSpeed(200);

  stepperY.setMaxSpeed(1000);
  stepperY.setAcceleration(500);
  stepperY.setSpeed(200);

  Serial.println("Stepper motors ready");
}

void setupLoadCell()
{
  Serial.println("Setting up load cell...");

  // Initialize HX711
  loadCell.begin(HX711_DT_PIN, HX711_SCK_PIN);

  // Set calibration factor (adjust based on your load cell)
  loadCell.set_scale(LOAD_CELL_CALIBRATION_FACTOR);

  // Tare (zero) the scale
  Serial.println("Taring load cell... Please ensure no weight is on the scale.");
  loadCell.tare(); // Reset the scale to 0

  Serial.println("Load cell ready");
}

void moveToPosition(float x, float y)
{
  // Check boundaries
  if (x < 0 || x > MAX_X_MM || y < 0 || y > MAX_Y_MM)
  {
    Serial.printf("Error: Position (%.1f, %.1f) out of bounds!\n", x, y);
    return;
  }

  // Convert mm to steps
  long stepsX = x * STEPS_PER_MM_X;
  long stepsY = y * STEPS_PER_MM_Y;

  // Set target positions
  stepperX.moveTo(stepsX);
  stepperY.moveTo(stepsY);

  // Update target position
  targetPos.x = x;
  targetPos.y = y;
  currentPos.isMoving = true;

  Serial.printf("Moving to (%.1f, %.1f) mm...\n", x, y);
}

bool isAtTarget()
{
  return !stepperX.isRunning() && !stepperY.isRunning();
}

int getServoForPosition(float x, float y)
{
  // Find the closest drop point
  float minDistance = 999999;
  int closestServo = -1;

  for (int i = 0; i < 9; i++)
  {
    float distance = sqrt(pow(x - dropPoints[i].x, 2) + pow(y - dropPoints[i].y, 2));
    if (distance < minDistance)
    {
      minDistance = distance;
      closestServo = dropPoints[i].servoIndex;
    }
  }

  // Only activate servo if we're very close to a drop point (within 5mm)
  if (minDistance <= 5.0)
  {
    return closestServo;
  }

  return -1; // No servo to activate
}

void activateServo(int servoIndex, float targetWeight)
{
  if (servoIndex >= 0 && servoIndex < 9)
  {
    Serial.printf("Activating servo %d with load cell verification (target: %.1f g)...\n", servoIndex, targetWeight);

    bool itemDropped = false;
    int retryCount = 0;

    // Retry loop
    while (!itemDropped && retryCount < MAX_RETRIES)
    {
      Serial.printf("Attempt %d: Opening above servo for 5s...\n", retryCount + 1);

      // Open above servo (180°) for 5 seconds
      pwmAbove.setPWM(servoChannelsAbove[servoIndex], 0, SERVO_MAX);
      delay(5000); // Keep open for 5 seconds

      // Close above servo (0°)
      pwmAbove.setPWM(servoChannelsAbove[servoIndex], 0, SERVO_MIN);

      Serial.printf("Above servo closed. Opening below servo for 5s...\n");

      // Open below servo (180°) for 5 seconds
      pwmBelow.setPWM(servoChannelsBelow[servoIndex], 0, SERVO_MAX);
      delay(5000); // Keep open for 5 seconds

      // Close below servo (0°)
      pwmBelow.setPWM(servoChannelsBelow[servoIndex], 0, SERVO_MIN);

      Serial.printf("Below servo closed. Checking weight...\n");

      // Check weight after both servos have operated
      float weight = readLoadCell();
      Serial.printf("Current weight: %.1f g\n", weight);

      if (isWeightSufficient(weight, targetWeight))
      {
        Serial.printf("Item detected! Weight: %.1f g (target: %.1f g)\n", weight, targetWeight);
        itemDropped = true;
      }
      else
      {
        Serial.printf("Insufficient weight (%.1f g, target: %.1f g). Retrying...\n", weight, targetWeight);
        retryCount++;
        delay(1000); // Wait before retry
      }
    }

    if (itemDropped)
    {
      Serial.printf("Servo %d activation complete - Item successfully caught.\n", servoIndex);
    }
    else
    {
      Serial.printf("Failed to catch item after %d attempts. Servo %d activation failed.\n", MAX_RETRIES, servoIndex);
    }
  }
}

void parseSerialCommand()
{
  static String inputBuffer = "";

  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (inputBuffer.length() > 0)
      {
        // Check for sequence command
        if (inputBuffer.equalsIgnoreCase("sequence") || inputBuffer.equalsIgnoreCase("start"))
        {
          startSequence();
        }
        else if (inputBuffer.indexOf(';') > 0)
        {
          // Parse sequence format: "1,300;5,100;9,200;"
          parseSequenceCommand(inputBuffer);
        }
        else
        {
          // Parse "X,Y" format
          int commaIndex = inputBuffer.indexOf(',');
          if (commaIndex > 0)
          {
            float x = inputBuffer.substring(0, commaIndex).toFloat();
            float y = inputBuffer.substring(commaIndex + 1).toFloat();

            moveToPosition(x, y);
          }
          else
          {
            Serial.println("Invalid format! Use: X,Y (e.g., 100,50), 'sequence' to start, or e.g., '1,300;5,100;9,200;' for custom sequence");
          }
        }
        inputBuffer = "";
      }
    }
    else
    {
      inputBuffer += c;
    }
  }
}

void printStatus()
{
  Serial.printf("Current Position: (%.1f, %.1f) mm", currentPos.x, currentPos.y);
  if (currentPos.isMoving)
  {
    Serial.printf(" -> Target: (%.1f, %.1f) mm", targetPos.x, targetPos.y);
  }
  Serial.println();
}

float readLoadCell()
{
  // Take multiple readings and average them for better accuracy
  float total = 0;
  for (int i = 0; i < LOAD_CELL_READINGS; i++)
  {
    total += loadCell.get_units();
    delay(10); // Small delay between readings
  }
  return total / LOAD_CELL_READINGS;
}

bool isWeightSufficient(float weight, float targetWeight)
{
  return weight >= (targetWeight - WEIGHT_TOLERANCE);
}

void parseSequenceCommand(String command)
{
  // Parse format: "1,300;5,100;9,200;"
  numSequencePoints = 0;
  int startIndex = 0;
  int endIndex = command.indexOf(';');

  while (endIndex >= 0 && numSequencePoints < 9)
  {
    String pointStr = command.substring(startIndex, endIndex);
    int commaIndex = pointStr.indexOf(',');

    if (commaIndex > 0)
    {
      int dropPointNum = pointStr.substring(0, commaIndex).toInt();
      float targetWeight = pointStr.substring(commaIndex + 1).toFloat();

      // Validate drop point number (1-9)
      if (dropPointNum >= 1 && dropPointNum <= 9)
      {
        int servoIndex = dropPointNum - 1; // Convert to 0-based index
        sequencePoints[numSequencePoints].x = dropPoints[servoIndex].x;
        sequencePoints[numSequencePoints].y = dropPoints[servoIndex].y;
        sequencePoints[numSequencePoints].servoIndex = servoIndex;
        sequencePoints[numSequencePoints].targetWeight = targetWeight;
        numSequencePoints++;

        Serial.printf("Added drop point %d (servo %d) with target weight %.1f g\n",
                      dropPointNum, servoIndex, targetWeight);
      }
      else
      {
        Serial.printf("Invalid drop point number: %d (must be 1-9)\n", dropPointNum);
      }
    }

    startIndex = endIndex + 1;
    endIndex = command.indexOf(';', startIndex);
  }

  if (numSequencePoints > 0)
  {
    Serial.printf("Sequence configured with %d points. Send 'sequence' to start.\n", numSequencePoints);
  }
  else
  {
    Serial.println("No valid drop points found in sequence command.");
  }
}

// ===== Sequence Functions =====
void startSequence()
{
  if (!sequenceRunning)
  {
    sequenceRunning = true;
    currentSequenceIndex = 0;
    Serial.println("Starting sequence...");
    moveToPosition(sequencePoints[currentSequenceIndex].x, sequencePoints[currentSequenceIndex].y);
  }
}

void handleSequence()
{
  if (sequenceRunning && !currentPos.isMoving && currentSequenceIndex < numSequencePoints)
  {
    // Activate servo for current point
    int servoIndex = sequencePoints[currentSequenceIndex].servoIndex;
    float targetWeight = sequencePoints[currentSequenceIndex].targetWeight;
    activateServo(servoIndex, targetWeight);

    // Move to next point
    currentSequenceIndex++;
    if (currentSequenceIndex < numSequencePoints)
    {
      Serial.printf("Moving to next point %d...\n", currentSequenceIndex + 1);
      moveToPosition(sequencePoints[currentSequenceIndex].x, sequencePoints[currentSequenceIndex].y);
    }
    else
    {
      Serial.println("Sequence complete!");
      sequenceRunning = false;
    }
  }
}
