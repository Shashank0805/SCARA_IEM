#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ==========================================
// USER CONFIGURATION SECTION
// ==========================================

// 1. ARM LENGTHS
const float L1 = 11.0; 
const float L2 = 13.0; 

// 2. SERVO CALIBRATION
// Offsets remain same to keep your workspace valid
const int OFFSET_BASE   = 45; 
const int OFFSET_ELBOW  = 90; 

// 3. PIN DEFINITIONS
const int PIN_BASE    = 0;  // 35kg Digital
const int PIN_ELBOW   = 15; // 35kg Digital
const int PIN_Z_AXIS  = 8;  // MG90s
const int PIN_GRIPPER = 7;  // MG90s

// 4. SERVO SETTINGS
#define SERVO_FREQ 50 
#define DIG_MIN 102 
#define DIG_MAX 512 
#define MG_MIN  130 
#define MG_MAX  490 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Global variables
float currentX = L1 + L2;
float currentY = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("SCARA Robot Initializing...");
  
  if (L1 == 0 || L2 == 0) {
    while(1); 
  }

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(100);

  // --- CUSTOM HOME POSITION (0, 240) ---
  Serial.println("Moving to Custom Home (Base:0, Middle:240)...");
  
  // Directly drive servos to your requested angles
  // This bypasses IK math and forces the physical position
  setServoPulse(PIN_BASE, 0);    
  setServoPulse(PIN_ELBOW, 240); 
  setServoPulse(PIN_Z_AXIS, 0);  // Pinion Up
  setServoPulse(PIN_GRIPPER, 0); // Gripper Open
  
  delay(2000); // Wait for it to reach home
}

void loop() {
  Serial.println("Starting Routine...");

  // 1. PICK (X=12, Y=5)
  moveToSmooth(12, 5, 0, false, 20); 
  delay(500);

  // Pinion Down
  moveSCARA(12, 5, 170, false); 
  delay(500);

  // Grip
  moveSCARA(12, 5, 170, true);
  delay(500);

  // Lift
  moveSCARA(12, 5, 0, true);
  delay(500);

  // 2. PLACE (X=12, Y=-5)
  moveToSmooth(5, -5, 0, true, 20);
  delay(500);

  // Pinion Down
  moveSCARA(5, -5, 170, true);
  delay(500);

  // Release
  moveSCARA(5, -5, 170, false);
  delay(500);

  // Return Home (Optional: You can uncomment the lines below to return to 0,240 after loop)
  /*
  setServoPulse(PIN_BASE, 0);
  setServoPulse(PIN_ELBOW, 240);
  delay(2000);
  */
  
  // Currently returns to standard math home, you can change this if you want
  moveSCARA(5, -5, 0, false);
  moveToSmooth(L1 + L2, 0, 0, false, 20);
  
  delay(3000); 
}

// ==========================================
// CORE FUNCTIONS
// ==========================================

void moveToSmooth(float targetX, float targetY, int targetZ, bool grip, int speed) {
  float startX = currentX;
  float startY = currentY;
  
  float dist = sqrt(pow(targetX - startX, 2) + pow(targetY - startY, 2));
  int steps = (int)dist * 2; 
  if (steps < 10) steps = 10;

  for (int i = 1; i <= steps; i++) {
    float interpX = startX + (targetX - startX) * (i / (float)steps);
    float interpY = startY + (targetY - startY) * (i / (float)steps);
    moveSCARA(interpX, interpY, targetZ, grip);
    delay(speed);
  }
}

void moveSCARA(float x, float y, int z_angle, bool gripper_closed) {
  float theta1, theta2;

  if (!calculateIK(x, y, theta1, theta2)) {
    Serial.println("Target out of Workspace!");
    return;
  }

  // APPLY OFFSET
  int val_base = theta1 + OFFSET_BASE; 
  int val_mid = theta2 + OFFSET_ELBOW; 

  // STRICT CONSTRAINTS
  if (val_base < 0) val_base = 0;
  if (val_base > 90) val_base = 90;

  if (val_mid < 90) val_mid = 90;
  if (val_mid > 240) val_mid = 240;

  // Z-Axis limit
  z_angle = constrain(z_angle, 0, 180);
  
  int gripper_val = gripper_closed ? 150 : 100;
  setServoPulse(PIN_BASE, val_base);
  setServoPulse(PIN_ELBOW, val_mid);
  setServoPulse(PIN_Z_AXIS, z_angle);
  setServoPulse(PIN_GRIPPER, gripper_val);

  currentX = x;
  currentY = y;
}

void setServoPulse(int pin, int angle) {
  int pulse;
  if (pin == PIN_BASE || pin == PIN_ELBOW) {
    angle = constrain(angle, 0, 270);
    pulse = map(angle, 0, 270, DIG_MIN, DIG_MAX);
  } else {
    angle = constrain(angle, 0, 180);
    pulse = map(angle, 0, 180, MG_MIN, MG_MAX);
  }
  pwm.setPWM(pin, 0, pulse);
}

bool calculateIK(float x, float y, float &theta1, float &theta2) {
  float r_sq = x*x + y*y;
  float r = sqrt(r_sq);
  
  if (r > (L1 + L2) || r < abs(L1 - L2)) return false;

  float cos_angle2 = (r_sq - L1*L1 - L2*L2) / (2 * L1 * L2);
  cos_angle2 = constrain(cos_angle2, -1.0, 1.0);
  float angle2_rad = acos(cos_angle2);

  float phi = atan2(L2 * sin(angle2_rad), L1 + L2 * cos(angle2_rad));
  float base_target_angle = atan2(y, x);
  
  float angle1_rad = base_target_angle - phi;

  theta1 = angle1_rad * 180.0 / PI;
  theta2 = angle2_rad * 180.0 / PI;

  return true;
}