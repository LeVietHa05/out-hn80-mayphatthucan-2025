#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
// #include <HX711.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ===== Pin Definitions =====
#define STEP_PIN_X 19
#define DIR_PIN_X 18
#define STEP_PIN_Y 25
#define DIR_PIN_Y 23
#define ENABLE_PIN_X 17
#define ENABLE_PIN_Y 16

#define LIMIT_SWITCH1_PIN 32 // check
#define LIMIT_SWITCH2_PIN 33 // check

// ===== HX711 Load Cell =====
#define HX711_DT_PIN 32  // Data pin
#define HX711_SCK_PIN 33 // Clock pin

// ===== System Configuration =====
#define STEPS_PER_ROUND 200
#define MICROSTEP 4 // MS2, MS1: 00: 1/8, 01: 1/32, 10: 1/64 11: 1/16
// Lead screw: Tr10x8 (8mm lead per revolution)
#define LEADSCREW 8
// Stepper: 200 steps/rev with 16 microstepping = 3200 steps/rev
// Steps per mm: 3200 / 8 = 400 steps per mm
#define STEPS_PER_MM_X STEPS_PER_ROUND *MICROSTEP / LEADSCREW // Tr10x8 lead screw with 16 microstepping
#define STEPS_PER_MM_Y STEPS_PER_ROUND *MICROSTEP / LEADSCREW // Tr10x8 lead screw with 16 microstepping
#define MAX_X_MM 560                                          // Maximum X travel in mm
#define MAX_Y_MM 310                                          // Maximum Y travel in mm
#define SERVO_MIN 150                                         // 0° position
#define SERVO_MAX 600                                         // 180° position

// ===== Load Cell Configuration =====
#define LOAD_CELL_CALIBRATION_FACTOR 1000.0 // Calibration factor (adjust based on your load cell)
#define WEIGHT_TOLERANCE 5.0                // Acceptable weight tolerance in grams
#define MAX_RETRIES 3                       // Maximum number of retry attempts
#define LOAD_CELL_READINGS 10               // Number of readings to average

#define SERVER "172.16.30.209"
#define PORT 3000
#define API "/api/claim/queue" // get the latest order from server

String currentQueueCode = ""; // queue code
int currentQueueID = 0;       // queue id
String scurLocations = "";    // string of locationlist
int locationList[] = {};      // list of drop point + 1

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
// HX711 loadCell;

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

DropPoint origin = {0, 0, 0};

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
    {560, 150, 0}, // Position 1 -> Servo 0
    {370, 150, 1}, // Position 2 -> Servo 1
    {190, 140, 2}, // Position 3 -> Servo 2
    {560, 75, 3},  // Position 4 -> Servo 3
    {370, 75, 4},  // Position 5 -> Servo 4
    {190, 75, 5},  // Position 6 -> Servo 5
    {560, 0, 6},   // Position 7 -> Servo 6
    {370, 0, 7},   // Position 8 -> Servo 7
    {190, 0, 8}    // Position 9 -> Servo 8
};

// ===== Function Declarations =====
void setupServos();
void setupSteppers();
// void setupLoadCell();
void moveToPosition(float x, float y);
void activateServo(int servoIndex, float targetWeight);
void printStatus();
// float readLoadCell();
bool isWeightSufficient(float weight, float targetWeight);
bool homeSteppersAdvanced(float homeSpeed = 2000, float maxSpeed = 2000,
                          int backoffDistance = 400, unsigned long timeout = 30000);
void fetchQueueFromServer();
int parseStringToSortedArray(const String &input, int output[], int maxSize);
void runQueue();

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
  Wire.begin(22, 21);

  // Setup components
  setupServos();
  setupSteppers();
  // setupLoadCell();

  // limit switch setup
  pinMode(LIMIT_SWITCH1_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH2_PIN, INPUT_PULLUP);

  homeSteppersAdvanced(2000);

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
  static unsigned long lastCheckServer = 0;
  if (millis() - lastCheckServer > 5000)
  {
    // fetchQueueFromServer();
    lastCheckServer = millis();
  }

  if (Serial.available())
  {
    int slot = Serial.readStringUntil('\n').toInt();
    if (slot >= 0 && slot <= 8)
    {
      DropPoint point = dropPoints[slot];
      moveToPosition(point.x, point.y);
    }
  }
}

void setupServos()
{
  Serial.println("Setting up servo system...");

  // Initialize above servos (PCA9685 at 0x40)
  pwmAbove.begin();
  pwmAbove.setPWMFreq(60); // 50Hz for servo motors

  // Initialize below servos (PCA9685 at 0x41)
  pwmBelow.begin();
  pwmBelow.setPWMFreq(60); // 50Hz for servo motors

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
  stepperX.setMaxSpeed(3000);     // steps per second
  stepperX.setAcceleration(1000); // steps per second²
  stepperX.setSpeed(200);
  stepperX.setEnablePin(ENABLE_PIN_X);
  stepperX.enableOutputs();

  stepperY.setMaxSpeed(3000);
  stepperY.setAcceleration(1000);
  stepperY.setSpeed(200);
  stepperY.setEnablePin(ENABLE_PIN_Y);
  stepperY.enableOutputs();

  digitalWrite(ENABLE_PIN_X, LOW);
  digitalWrite(ENABLE_PIN_Y, LOW);

  Serial.println("Stepper motors ready");
}

// void setupLoadCell()
// {
//   Serial.println("Setting up load cell...");

//   // Initialize HX711
//   loadCell.begin(HX711_DT_PIN, HX711_SCK_PIN);

//   // Set calibration factor (adjust based on your load cell)
//   loadCell.set_scale(LOAD_CELL_CALIBRATION_FACTOR);

//   // Tare (zero) the scale
//   Serial.println("Taring load cell... Please ensure no weight is on the scale.");
//   loadCell.tare(); // Reset the scale to 0

//   Serial.println("Load cell ready");
// }

// move with blocking untill done
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
  stepperX.moveTo(-stepsX);
  stepperY.moveTo(stepsY);

  Serial.printf("Moving to (%.1f, %.1f) mm...\n", x, y);

  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
  {
    stepperX.run();
    stepperY.run();
  }
}

void activateServo(int servoIndex, float targetWeight)
{
  if (servoIndex >= 0 && servoIndex < 9)
  {
    Serial.printf("Activating servo %d with load cell verification (target: %.1f g)...\n", servoIndex, targetWeight);

    bool itemDropped = false;
    int retryCount = 0;

    // Retry loop
    // while (!itemDropped && retryCount < MAX_RETRIES)
    // {
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
    delay(2000);
    Serial.printf("Below servo closed. Checking weight...\n");
  }
}

void printStatus()
{
  // Serial.printf("Current Position: (%.1f, %.1f) mm", currentPos.x, currentPos.y);
  // if (currentPos.isMoving)
  // {
  //   Serial.printf(" -> Target: (%.1f, %.1f) mm", targetPos.x, targetPos.y);
  // }
  Serial.println();
}

bool isWeightSufficient(float weight, float targetWeight)
{
  return weight >= (targetWeight - WEIGHT_TOLERANCE);
}

bool homeSteppersAdvanced(float homeSpeed, float maxSpeed,
                          int backoffDistance, unsigned long timeout)
{
  Serial.println("=== Bắt đầu quá trình Homing ===");

  unsigned long startTime = millis();
  bool home1 = false;
  bool home2 = false;

  // Thiết lập tốc độ homing
  stepperX.setMaxSpeed(maxSpeed);
  stepperX.setSpeed(homeSpeed);
  stepperY.setMaxSpeed(maxSpeed);
  stepperY.setSpeed(homeSpeed);

  // Bắt đầu di chuyển về phía công tắc
  stepperX.move(1000000);
  stepperY.move(-1000000);

  while (!(home1 && home2))
  {
    // Kiểm tra timeout
    if (millis() - startTime > timeout)
    {
      Serial.println("Lỗi: Timeout trong quá trình homing!");
      stepperX.stop();
      stepperY.stop();
      return false;
    }

    // Kiểm tra công tắc
    if (!home1)
    {
      if (digitalRead(LIMIT_SWITCH1_PIN) == LOW)
      {
        stepperX.stop();
        stepperX.setCurrentPosition(0);
        home1 = true;
        Serial.println("✓ Stepper 1: Đã về home");
      }
      else
      {
        stepperX.run(); // Dùng runSpeed() để kiểm soát tốc độ
      }
    }

    if (!home2)
    {
      if (digitalRead(LIMIT_SWITCH2_PIN) == LOW)
      {
        stepperY.stop();
        stepperY.setCurrentPosition(0);
        home2 = true;
        Serial.println("✓ Stepper 2: Đã về home");
      }
      else
      {
        stepperY.run();
      }
    }

    // Ngắn delay để tránh đọc switch quá nhanh
    delay(1);
  }

  Serial.println("Đang di chuyển ra khỏi công tắc...");

  // Di chuyển ngược lại với tốc độ chậm hơn
  stepperX.setSpeed(homeSpeed / 2);
  stepperY.setSpeed(homeSpeed / 2);

  stepperX.move(-backoffDistance);
  stepperY.move(backoffDistance);

  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
  {
    stepperX.runSpeed();
    stepperY.runSpeed();
  }

  // Đặt lại vị trí zero
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // Khôi phục tốc độ mặc định
  stepperX.setMaxSpeed(3000);
  stepperY.setMaxSpeed(3000);

  Serial.println("=== Homing hoàn tất thành công ===");
  return true;
}

void fetchQueueFromServer()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    String url = "http://" + String(SERVER) + ":" + String(PORT) + String(API);
    http.begin(url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      String response = http.getString();
      Serial.println("GET Response: " + response);
      // Parse the response to get the state
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, response);

      if (error)
      {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }
      bool success = doc["success"]; // bool
      if (success)
      {
        JsonObject data = doc["data"];
        currentQueueID = data["id"];                                                // 3
        currentQueueCode = data["selection_code"].as<String>();                     // "0K9UP7"
        const char *data_dishes = data["dishes"];                                   // "[21,4,40]"
        parseStringToSortedArray(data["locations"].as<String>(), locationList, 10); // "[5,3,7]"
        const char *data_status = data["status"];                                   // "pending"
        int data_attempts = data["attempts"];                                       // 0
        const char *data_created_at = data["created_at"];                           // "2025-12-15 11:11:58"
        Serial.println(locationList[0]);
        Serial.println(locationList[1]);
        Serial.println(locationList[2]);
        runQueue();
      }
    }
    else
    {
      Serial.println("Error getting: " + String(httpResponseCode));
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi not connected");
  }
}

int parseStringToSortedArray(const String &input, int output[], int maxSize)
{
  int count = 0;

  if (input.length() == 0 || maxSize == 0)
  {
    return 0;
  }

  // Chuyển String sang char array
  char buffer[input.length() + 1];
  input.toCharArray(buffer, sizeof(buffer));

  // Dùng strtok để tách chuỗi
  char *token = strtok(buffer, "[], ");

  while (token != NULL && count < maxSize)
  {
    output[count] = atoi(token);
    count++;
    token = strtok(NULL, "[], ");
  }

  // Sắp xếp (dùng qsort cho nhanh)
  if (count > 1)
  {
    qsort(output, count, sizeof(int), [](const void *a, const void *b)
          { return (*(int *)a - *(int *)b); });
  }

  return count;
}

void runQueue()
{
  Serial.println("runing for queueid: " + String(currentQueueID) + " with code" + currentQueueCode);

  for (int i = 0; i < 3; i++)
  {
    DropPoint cur = dropPoints[locationList[i] - 1];
    moveToPosition(cur.x, cur.y);
    activateServo(cur.servoIndex, 1.0);
  }

  moveToPosition(origin.x, origin.y);
}

void postQueueComplete()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    String url = "http://" + String(SERVER) + ":" + String(PORT) + String(API) + "?queueid=" + String(currentQueueID);
    http.begin(url);
    int httpResponseCode = http.POST("");

    if (httpResponseCode > 0)
    {
      String response = http.getString();
      Serial.println(response);
    }
    else
    {
      Serial.println("Error getting: " + String(httpResponseCode));
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi not connected");
  }
}
