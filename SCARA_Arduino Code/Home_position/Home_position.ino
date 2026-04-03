#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---- SERVO CONFIG ----
const int PWM_FREQ = 60;         // PWM frequency for PCA9685
const int SERVO_RES = 4096;      // 12-bit resolution
const float US_MIN = 500.0;
const float US_MAX = 2500.0;
const float SERVO_JOINT_MAX = 180.0;

// Servo channels
const int CH_SERVO1 = 0;  // Shoulder
const int CH_SERVO2 = 15;  // Elbow

// Home angles (set these as you want)
float HOME_SHOULDER_ANGLE = 45.0;
float HOME_ELBOW_ANGLE = 0.0;

// Current joint positions
float currentShoulder = 0.0;
float currentElbow = 0.0;

// Convert degrees to PWM pulse length
int jointDegToPulse(float jointDeg) {
  jointDeg = constrain(jointDeg, 0.0, SERVO_JOINT_MAX);
  float us = US_MIN + (jointDeg / SERVO_JOINT_MAX) * (US_MAX - US_MIN);
  float ticks = (us * PWM_FREQ * SERVO_RES) / 1000000.0;
  return int(ticks);
}

// Move servos to specified angles with interpolation
void moveToAngles(float startShoulder, float startElbow,
                  float endShoulder, float endElbow,
                  int steps = 10, int stepDelay = 15) {
  for (int i = 1; i <= steps; i++) {
    float interpShoulder = startShoulder + (endShoulder - startShoulder) * i / steps;
    float interpElbow = startElbow + (endElbow - startElbow) * i / steps;

    pwm.setPWM(CH_SERVO1, 0, jointDegToPulse(interpShoulder));
    pwm.setPWM(CH_SERVO2, 0, jointDegToPulse(interpElbow));

    delay(stepDelay);
  }
}

// Move both arms to the home position
void goHome(float &currentShoulder, float &currentElbow) {
  Serial.println("Moving to Home Position");
  moveToAngles(currentShoulder, currentElbow,
               HOME_SHOULDER_ANGLE, HOME_ELBOW_ANGLE,
               30, 20);
  currentShoulder = HOME_SHOULDER_ANGLE;
  currentElbow = HOME_ELBOW_ANGLE;
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // Initialize current angles if needed (e.g., read from sensors or start at 0)
  currentShoulder = 0.0;
  currentElbow = 0.0;

  goHome(currentShoulder, currentElbow);
}

void loop() {
  // No loop action needed for just homing
}
