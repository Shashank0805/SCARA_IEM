#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Servo config ---
const int PWM_FREQ = 60;
const int SERVO_RES = 4096;
const float US_MIN = 500.0;
const float US_MAX = 2500.0;

const int CH_SERVO1 = 0; // Shoulder
const int CH_SERVO2 = 15; // Elbow
const int CH_PEN    = 8; // Pen servo

const float PEN_UP_ANGLE   = 0.0;
const float PEN_DOWN_ANGLE = 180.0;

// Current joint positions
float currentShoulder = 0.0;
float currentElbow    = 0.0;
float currentPen      = PEN_UP_ANGLE;

// Convert deg → PWM
int degToPulse(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  float us = US_MIN + (deg / 180.0) * (US_MAX - US_MIN);
  return (int)((us * PWM_FREQ * SERVO_RES) / 1000000.0);
}

// Smooth move pen
void movePen(float target, int steps=10, int delayMs=15) {
  for(int i=1; i<=steps; i++){
    float interp = currentPen + (target - currentPen) * i / steps;
    pwm.setPWM(CH_PEN, 0, degToPulse(interp));
    delay(delayMs);
  }
  currentPen = target;
}

// Move arm
void moveToAngles(float t1, float t2, int steps=100, int delayMs=15) {
  for(int i=1; i<=steps; i++){
    float interpT1 = currentShoulder + (t1 - currentShoulder) * i / steps;
    float interpT2 = currentElbow    + (t2 - currentElbow) * i / steps;
    pwm.setPWM(CH_SERVO1, 0, degToPulse(interpT1));
    pwm.setPWM(CH_SERVO2, 0, degToPulse(interpT2));
    delay(delayMs);
  }
  currentShoulder = t1;
  currentElbow    = t2;
}

// Process serial
void processCommand(String cmd){
  cmd.trim();
  if(cmd.length()==0) return;

  int first = cmd.indexOf(',');
  int second = cmd.indexOf(',', first+1);
  if(first>=0 && second>first){
    float t1 = cmd.substring(0, first).toFloat();
    float t2 = cmd.substring(first+1, second).toFloat();
    float pen = cmd.substring(second+1).toFloat();

    // Only move pen if command does not says so
    if(pen != currentPen) movePen(pen, 10, 15);
    moveToAngles(t1, t2, 5, 10);
  }
}

void setup(){
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // Start with pen up
  movePen(PEN_UP_ANGLE, 20, 20);
  delay(300);

  // Move to home
  

  // ✅ DO NOT lower pen here
}

void loop(){
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}