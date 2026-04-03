#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ==================================================
// CONFIGURATION
// ==================================================

const int PWM_FREQ = 60;     // Standard for analog servos
const float US_MIN = 500.0;  // Adjust if servos buzz at 0
const float US_MAX = 2500.0; // Adjust if servos buzz at 180
const int SERVO_RES = 4096;

// --- PCA9685 Channel Mapping ---
const int CH_SHOULDER = 0;   // Arm Segment 1
const int CH_ELBOW    = 15;   // Arm Segment 2 
const int CH_Z        = 8;   // Up/Down Servo
const int CH_GRIP     = 7;   // Gripper Servo

// --- Current State Tracking ---
// We initialize these to 90 just for math, 
// but we WON'T write to motors until Python sends data.
float currentShoulder = 90.0; 
float currentElbow    = 90.0;
float currentZ        = 0.0; 
float currentG        = 90.0;

// ==================================================
// HELPER FUNCTIONS
// ==================================================

// Convert Degrees to PWM Pulse
int degToPulse(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  float us = US_MIN + (deg / 180.0) * (US_MAX - US_MIN);
  return (int)((us * PWM_FREQ * SERVO_RES) / 1000000.0);
}

// Set a servo immediately (for Z and Gripper)
void setServo(int channel, float deg) {
  pwm.setPWM(channel, 0, degToPulse(deg));
}

// Smoothly move the main arms (Interpolation)
void moveArmsSmooth(float targetT1, float targetT2, int steps=40, int delayMs=10) {
  // If the change is very small, just go there instantly
  if (abs(targetT1 - currentShoulder) < 1.0 && abs(targetT2 - currentElbow) < 1.0) {
    setServo(CH_SHOULDER, targetT1);
    setServo(CH_ELBOW, targetT2);
    currentShoulder = targetT1;
    currentElbow = targetT2;
    return;
  }

  for(int i=1; i<=steps; i++){
    float interpT1 = currentShoulder + (targetT1 - currentShoulder) * i / steps;
    float interpT2 = currentElbow    + (targetT2 - currentElbow) * i / steps;
    
    setServo(CH_SHOULDER, interpT1);
    setServo(CH_ELBOW, interpT2);
    delay(delayMs);
  }
  
  // Ensure final position is exact
  currentShoulder = targetT1;
  currentElbow    = targetT2;
  setServo(CH_SHOULDER, currentShoulder);
  setServo(CH_ELBOW, currentElbow);
}

// ==================================================
// COMMAND PARSING
// ==================================================

void processCommand(String cmd){
  cmd.trim();
  if(cmd.length() == 0) return;

  // Expected Format: T1, T2, Z, G
  // Example: 90.0, 45.0, 180, 180

  int ind1 = cmd.indexOf(',');
  int ind2 = cmd.indexOf(',', ind1+1);
  int ind3 = cmd.indexOf(',', ind2+1);

  if(ind1 > 0 && ind2 > 0 && ind3 > 0) {
    float t1 = cmd.substring(0, ind1).toFloat();
    float t2 = cmd.substring(ind1+1, ind2).toFloat();
    float z  = cmd.substring(ind2+1, ind3).toFloat();
    float g  = cmd.substring(ind3+1).toFloat();

    // 1. Handle Z and Gripper actions separately (Fast/Instant)
    // In a Pick/Place logic, usually Z/G happens before or after arm move.
    // However, since Python sends the specific state for this specific step,
    // we update Z and G immediately, then move arms.
    
    if(z != currentZ) {
       setServo(CH_Z, z);
       currentZ = z;
       delay(100); // Give Z time to move
    }

    if(g != currentG) {
       setServo(CH_GRIP, g);
       currentG = g;
       delay(100); // Give Gripper time
    }

    // 2. Move Arms Smoothly
    moveArmsSmooth(t1, t2);
  }
}

// ==================================================
// SETUP & LOOP
// ==================================================

void setup(){
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // --- SAFE STARTUP ---
  // 1. Retract Z (Lift up) so we don't hit the board
  setServo(CH_Z, 0); 
  currentZ = 0;

  // 2. Open Gripper
  setServo(CH_GRIP, 90); 
  currentG = 90;

  // 3. DO NOT MOVE ARMS
  // We leave CH_SHOULDER and CH_ELBOW alone. 
  // They will engage only when the first command arrives.
  // Note: If your servos are powered, they might be limp until then.
}

void loop(){
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
    Serial.println("OK"); // Ack to Python (optional but good for debugging)
  }
}
