#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CONFIGURATION ---
const int PIN_BASE = 0;    
const int PIN_MIDDLE = 15; 

// --- 270 DEGREE SERVO SETTINGS ---
#define SERVOMIN  102 
#define SERVOMAX  512 
#define SERVO_FREQ 50 

void setup() {
  Serial.begin(9600);
  Serial.println("--- DUAL ACTUATOR TESTER (FIXED) ---");
  Serial.println("Enter: BaseAngle, MiddleAngle (e.g. 45, 120)");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    // peek() looks at the next character without removing it
    // If the next character is not a number (like a newline), we discard it.
    char c = Serial.peek();
    
    if (!(c >= '0' && c <= '9') && c != '-') {
      Serial.read(); // Discard garbage (newlines, spaces)
      return;        // Restart loop
    }

    // Only try to parse if we are sure there is a number
    int base_angle = Serial.parseInt(); 
    int mid_angle = Serial.parseInt();

    // Only move if we actually got two valid updates (parseInt defaults to 0 on timeout)
    // We check if the input was actually valid (not just a timeout zero)
    if (Serial.read() == '\n' || true) { // simple flush trigger
       // Safety Check
       if (base_angle >= 0 && base_angle <= 270 && mid_angle >= 0 && mid_angle <= 270) {
         moveDualServos(base_angle, mid_angle);
       } 
       // Note: We don't print "Error" here for 0,0 cases caused by timeouts to keep the monitor clean
    }
  }
}

void moveDualServos(int base, int mid) {
  int pulse1 = map(base, 0, 270, SERVOMIN, SERVOMAX);
  int pulse2 = map(mid, 0, 270, SERVOMIN, SERVOMAX);
  
  pwm.setPWM(PIN_BASE, 0, pulse1);
  pwm.setPWM(PIN_MIDDLE, 0, pulse2);
  
  Serial.print("Moved to -> Base: ");
  Serial.print(base);
  Serial.print(" | Middle: ");
  Serial.println(mid);
}