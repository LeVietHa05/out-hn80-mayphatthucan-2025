#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

#define dw digitalWrite
#define dr digitalRead

// ===== Servo (PCA9685) =====
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_MIN 150                   // góc 0°
#define SERVO_MAX 600                   // góc 180°
int servoChannels[5] = {0, 1, 2, 3, 4}; // 5 servo nối vào kênh 0~4

// ===== Stepper (TMC2209) =====
#define STEP_PIN1 18
#define DIR_PIN1 19
#define STEP_PIN2 23
#define DIR_PIN2 25
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

void turnServo(int servoChanel, int pos)
{
  pwm.setPWM(servoChanel, 0, pos);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Khởi động ...");

  // Servo init
  pwm.begin();
  pwm.setPWMFreq(50); // Tần số 50Hz phù hợp với servo

  // Stepper init
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  dw(STEP_PIN1, LOW);
  dw(STEP_PIN2, LOW);
  dw(DIR_PIN1, LOW);
  dw(DIR_PIN2, LOW);
  stepper1.setMaxSpeed(1000); // bước/giây
  stepper1.setAcceleration(300);
  stepper1.setSpeed(100); // Set initial speed
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(300);
  stepper2.setSpeed(100); // Set initial speed

  stepper1.moveTo(0);
  stepper2.moveTo(0);

  delay(1000);
}

void loop()
{
  stepper1.run(); // Non-blocking run
  stepper2.run(); // Non-blocking run

  
}