// ---- START ROBOT CONFIGURATION ----
// Robot Configuration:
// [Name]               [Type]             [Port(s)/Pin(s)]          [Functionality]
// DRIVETRAIN_LEFT_1    PWM Output         9                         Left motor forward
// DRIVETRAIN_LEFT_2    PWM Output         8                         Left motor reverse
// DRIVETRAIN_RIGHT_1   PWM Output         10                        Right motor forward
// DRIVETRAIN_RIGHT_2   PWM Output         11                        Right motor reverse
// LINEAR_LEFT_1        PWM Output         12                        Left lift up
// LINEAR_LEFT_2        PWM Output         13                        Left lift down
// LINEAR_RIGHT_1       PWM Output         14                        Right lift up
// LINEAR_RIGHT_2       PWM Output         15                        Right lift down
// SERVO_INTAKE         Servo (360°)       7                         Intake rotate in/out
// SERVO_UP             Servo (180°)       5                         Block/unblock ball slots
// SERVO_OUTTAKE        Servo (360°)       6                         Outtake rotation
// SERVO_HOLDER         Servo (180°)       2                         Ball holder rotation
// LIMIT_SWITCH_PIN     Digital Input      25                        Detects ball in intake
// LIMIT_SWITCH_TOP_PIN Digital Input      32                        Detects top lift position
// Controller           PS2X Controller    12,13,14,15               Wireless robot control
//
// Button Mapping:
// [Button]             [Functionality]
// PSB_START            Start the match
// L1                   Lift up
// L2                   Lift down
// PINK                 Holder to 0° (release)
// GREEN                Holder to 135° (hold)
// R1                   Block 2 balls
// R2                   Unblock 2 balls
// PAD_LEFT             Unblock left ball
// PAD_RIGHT            Unblock right ball
// BLUE                 Intake close (clockwise)
// RED                  Intake open (counter-clockwise)
// PAD_UP               Outtake counter-clockwise
// PAD_DOWN             Outtake clockwise
// Analog LY/RX         Drivetrain control (forward/backward + rotation)
// ---- END ROBOT CONFIGURATION ----

#include <Wire.h>
#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>

// CONDITIONS //
#define SERVOMIN 110
#define SERVOMAX 600
#define SERVO_STOP 360
#define SERVO_0_DEGREE 110 

// LIMIT SWITCH //
#define LIMIT_SWITCH_PIN 25
#define LIMIT_SWITCH_TOP_PIN 32  

// DRIVETRAIN //
#define DRIVETRAIN_LEFT_1 9
#define DRIVETRAIN_LEFT_2 8
#define DRIVETRAIN_RIGHT_1 10
#define DRIVETRAIN_RIGHT_2 11

// LINEAR // 
#define LINEAR_LEFT_1 12
#define LINEAR_LEFT_2 13
#define LINEAR_RIGHT_1 14
#define LINEAR_RIGHT_2 15

// SERVO //
#define SERVO_INTAKE 7     // 360°
#define SERVO_UP 5         // 180°
#define SERVO_OUTTAKE 6    // 360°
#define SERVO_HOLDER 2    // 180°

const int LIFT_PWM = 4095;
const int angles[] = {0, 90, 120};
int angleUp = 50;
int angleDown = 157;
int angleLeft = 100;
int angleRight = 2;
int r2PressCount = 0;
int intakeDirection = 0; // 1: CW, -1: CCW, 0: stop
bool topSwitchLatched = false;
bool canOpenIntake = true;
bool prevLimitSwitchState = HIGH;
unsigned long intakeStartTime = 0;
unsigned long topSwitchTriggeredTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;

// Set up before match
void waitForStart() {
  Serial.println("Waiting for you...");

  pwm.setPin(DRIVETRAIN_LEFT_1, 0);
  pwm.setPin(DRIVETRAIN_LEFT_2, 0);
  pwm.setPin(DRIVETRAIN_RIGHT_1, 0);
  pwm.setPin(DRIVETRAIN_RIGHT_2, 0);

  pwm.setPin(LINEAR_LEFT_1, 4095);
  pwm.setPin(LINEAR_LEFT_2, 4095);
  pwm.setPin(LINEAR_RIGHT_1, 4095);
  pwm.setPin(LINEAR_RIGHT_2, 4095);

  int angleUp_before = 180;
  int pulseUp_before = map(angleUp_before, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(SERVO_UP, 0, pulseUp_before);

  while (true) {
    ps2x.read_gamepad();
    if (ps2x.ButtonPressed(PSB_START)) break;
    delay(50);
  }

  Serial.println("FPT_CT1 khởi động");

  int angleUP_after = 210;
  int pulseUP_after = map(angleUP_after, 0, 270, SERVOMIN, SERVOMAX); 
  pwm.setPWM(SERVO_UP, 0, pulseUP_after);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  Wire.setClock(400000);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  
  pinMode(LIMIT_SWITCH_TOP_PIN, INPUT_PULLUP);

  int error = -1;
  while (error != 0) {
    error = ps2x.config_gamepad(14, 13, 15, 12, true, true); // CLK, CMD, SEL, DAT
    delay(100);
  }
  Serial.println("PS2 Connected!");

  waitForStart();
}

void loop() {
  ps2x.read_gamepad();
  static bool climbing = false;
  static unsigned long climbStartTime = 0;
  static bool autoLowering = false;
  static unsigned long autoLowerStartTime = 0;
  bool controllerConnected = ps2x.Analog(PSS_LY) != 0 || ps2x.Analog(PSS_RX) != 0 || ps2x.Analog(PSS_RY) != 0;
  static bool wasDisconnected = false;
  if (!controllerConnected) {
    if (!wasDisconnected) {
      Serial.println("Disconnect ự ự");

      pwm.setPin(DRIVETRAIN_LEFT_1, 0);
      pwm.setPin(DRIVETRAIN_LEFT_2, 0);
      pwm.setPin(DRIVETRAIN_RIGHT_1, 0);
      pwm.setPin(DRIVETRAIN_RIGHT_2, 0);

      pwm.setPin(LINEAR_LEFT_1, 4095);
      pwm.setPin(LINEAR_LEFT_2, 4095);
      pwm.setPin(LINEAR_RIGHT_1, 4095);
      pwm.setPin(LINEAR_RIGHT_2, 4095);

      wasDisconnected = true;
    }
    delay(100); 
    return; 
  } else if (wasDisconnected) {
    Serial.println("Connect hẹ hẹ");
    wasDisconnected = false;
  }

  // DRIVETRAIN CONTROL //
  int ly = ps2x.Analog(PSS_LY);  
  int rx = ps2x.Analog(PSS_RX);  

  float y_raw = -(ly - 128);
  float r = -(rx - 128);

  // Deadzone
  const int dz = 10;
  y_raw = abs(y_raw) < dz ? 0 : y_raw;
  r = abs(r) < dz ? 0 : r;

  // Smooth
  static float y_smooth = 0;
  float alpha = 0.15;
  y_smooth = y_smooth + alpha * (y_raw - y_smooth);

  if (y_smooth > 70) r *= ((y_smooth - 50) / 100.0 + 1);
  else if (y_smooth < -70) r *= ((-70 - y_smooth) / 100.0 + 1);

  float left = y_smooth + r * 0.7;
  float right = y_smooth - r * 0.7;
  left = constrain(left, -127, 127);
  right = constrain(right, -127, 127);

  const int MAX_PWM = 4095;

  int left_pwm1 = left > 0 ? map(left, 0, 127, 0, MAX_PWM) : 0;
  int left_pwm2 = left < 0 ? map(-left, 0, 127, 0, MAX_PWM) : 0;
  int right_pwm1 = right > 0 ? map(right, 0, 127, 0, MAX_PWM) : 0;
  int right_pwm2 = right < 0 ? map(-right, 0, 127, 0, MAX_PWM) : 0;

  pwm.setPWM(DRIVETRAIN_LEFT_1, 0, left_pwm1);
  pwm.setPWM(DRIVETRAIN_LEFT_2, 0, left_pwm2);
  pwm.setPWM(DRIVETRAIN_RIGHT_1, 0, right_pwm1);
  pwm.setPWM(DRIVETRAIN_RIGHT_2, 0, right_pwm2);

  // LINEAR CONTROL //
  bool manualLift = false;
  if (ps2x.Button(PSB_L1)) {
    manualLift = true;
    autoLowering = false;
    // Limit switch
    if (digitalRead(LIMIT_SWITCH_TOP_PIN) == HIGH) {
      topSwitchLatched = false;
      pwm.setPin(LINEAR_LEFT_1, LIFT_PWM);
      pwm.setPin(LINEAR_LEFT_2, 0);
      pwm.setPin(LINEAR_RIGHT_1, LIFT_PWM);
      pwm.setPin(LINEAR_RIGHT_2, 0);
    }
    else {
      // Pushed
      if (!topSwitchLatched) {
        topSwitchLatched = true;
        topSwitchTriggeredTime = millis();
        pwm.setPin(LINEAR_LEFT_1, 4095);
        pwm.setPin(LINEAR_LEFT_2, 4095);
        pwm.setPin(LINEAR_RIGHT_1, 4095);
        pwm.setPin(LINEAR_RIGHT_2, 4095);
      }
      else {
        // After 2 seconds
        if (millis() - topSwitchTriggeredTime >= 2000) {
          pwm.setPin(LINEAR_LEFT_1, LIFT_PWM);
          pwm.setPin(LINEAR_LEFT_2, 0);
          pwm.setPin(LINEAR_RIGHT_1, LIFT_PWM);
          pwm.setPin(LINEAR_RIGHT_2, 0);
        }
        else {
          // In 2 seconds
          pwm.setPin(LINEAR_LEFT_1, 4095);
          pwm.setPin(LINEAR_LEFT_2, 4095);
          pwm.setPin(LINEAR_RIGHT_1, 4095);
          pwm.setPin(LINEAR_RIGHT_2, 4095);
        }
      }
    }
  }
  else if (ps2x.Button(PSB_L2)) {
    manualLift = true;
    autoLowering = false;
    pwm.setPin(LINEAR_LEFT_1, 0);
    pwm.setPin(LINEAR_LEFT_2, LIFT_PWM);
    pwm.setPin(LINEAR_RIGHT_1, 0);
    pwm.setPin(LINEAR_RIGHT_2, LIFT_PWM);
  }
  else if (autoLowering && millis() - autoLowerStartTime <= 3200) {
    pwm.setPin(LINEAR_LEFT_1, 0);
    pwm.setPin(LINEAR_LEFT_2, LIFT_PWM);
    pwm.setPin(LINEAR_RIGHT_1, 0);
    pwm.setPin(LINEAR_RIGHT_2, LIFT_PWM);
  }
  else {
    autoLowering = false;
    pwm.setPin(LINEAR_LEFT_1, 4095);
    pwm.setPin(LINEAR_LEFT_2, 4095);
    pwm.setPin(LINEAR_RIGHT_1, 4095);
    pwm.setPin(LINEAR_RIGHT_2, 4095);
  }
  // Auto lift down
  if (ps2x.ButtonPressed(PSB_L3)) {
    autoLowering = true;
    autoLowerStartTime = millis();
  }

  // HOLDER CONTROL //
  if (ps2x.ButtonPressed(PSB_GREEN)) {
    int pulse = map(135, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_HOLDER, 0, pulse);
  }

  // Limit switch application
  static bool holderReset = false;

  if (digitalRead(LIMIT_SWITCH_TOP_PIN) == LOW && !holderReset) {
    int pulse = map(0, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_HOLDER, 0, pulse);
    holderReset = true;
  } else if (digitalRead(LIMIT_SWITCH_TOP_PIN) == HIGH) {
    holderReset = false; 
  }

  // Auto lifting and hold
  static bool liftAfterR3Active = false;

  if (ps2x.ButtonPressed(PSB_R3)) {
    liftAfterR3Active = true;
    pwm.setPWM(SERVO_HOLDER, 0, map(135, 0, 180, SERVOMIN, SERVOMAX));
  }

  if (liftAfterR3Active) {
    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_L2)) {
      liftAfterR3Active = false;
    } else {
      // Auto lifting
      if (digitalRead(LIMIT_SWITCH_TOP_PIN) == HIGH) {
        pwm.setPin(LINEAR_LEFT_1, LIFT_PWM);
        pwm.setPin(LINEAR_LEFT_2, 0);
        pwm.setPin(LINEAR_RIGHT_1, LIFT_PWM);
        pwm.setPin(LINEAR_RIGHT_2, 0);
      } else {
        // Auto hold
        pwm.setPin(LINEAR_LEFT_1, 4095);
        pwm.setPin(LINEAR_LEFT_2, 4095);
        pwm.setPin(LINEAR_RIGHT_1, 4095);
        pwm.setPin(LINEAR_RIGHT_2, 4095);
        pwm.setPWM(SERVO_HOLDER, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
        liftAfterR3Active = false;
      }
    }
  }

  // D-PAD CONTROLL //
  // Block 2 balls
  if (ps2x.ButtonPressed(PSB_R1)) {
    int pulse = map(angleUp, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_UP, 0, pulse);
  }
  // Unblock 2 balls
  if (ps2x.ButtonPressed(PSB_R2)) {
    int pulse = map(angleDown, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_UP, 0, pulse);
  }

  // Unblock left ball
  if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
    int pulse = map(angleLeft, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_UP, 0, pulse);
  }

  // Unblock right ball
  if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
    int pulse = map(angleRight, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(SERVO_UP, 0, pulse);
  }

  // INTAKE //
  // Close intake
  if (ps2x.ButtonPressed(PSB_BLUE)) {
    intakeDirection = 1;
    intakeStartTime = millis(); 
    canOpenIntake = true;
  }
  
  // Stop intake
  if (ps2x.ButtonPressed(PSB_PINK)) {
    pwm.setPWM(SERVO_INTAKE, 0, SERVO_STOP);
    intakeDirection = 0;
  }

  // Open intake
  if (canOpenIntake && ps2x.ButtonPressed(PSB_RED)) {
    intakeDirection = -1;
    intakeStartTime = millis();
    canOpenIntake = false;  // Lock open
  }

  // Close intake logic
  if (intakeDirection == 1) {
    if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
      pwm.setPWM(SERVO_INTAKE, 0, SERVOMIN);  
      if (millis() - intakeStartTime >= 2500) {
        pwm.setPWM(SERVO_INTAKE, 0, SERVO_STOP);  
        intakeDirection = 0;
      }
    } else {
      pwm.setPWM(SERVO_INTAKE, 0, SERVO_STOP);  
      intakeDirection = 0;
    }
  }

  // Open intake logic
  else if (intakeDirection == -1) {
    if (millis() - intakeStartTime <= 1350) {
      pwm.setPWM(SERVO_INTAKE, 0, SERVOMAX); 
    } else {
      pwm.setPWM(SERVO_INTAKE, 0, SERVO_STOP);
      intakeDirection = 0;
    }
  }

  // OUTTAKE//
  if (ps2x.Button(PSB_PAD_UP)) {
    pwm.setPWM(SERVO_OUTTAKE, 0, SERVOMAX); // CCW
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    pwm.setPWM(SERVO_OUTTAKE, 0, SERVOMIN); // CW
  } else {
    pwm.setPWM(SERVO_OUTTAKE, 0, SERVO_STOP);
  }

  //AUTO CLIMB
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    climbing = true;
    climbStartTime = millis();
  }

  if (climbing && (ps2x.Button(PSB_L1) || ps2x.Button(PSB_L2))) {
    climbing = false;
  }

  if (climbing) {
    if (millis() - climbStartTime <= 5500) {
  
      pwm.setPWM(DRIVETRAIN_LEFT_1, 0, 0);
      pwm.setPWM(DRIVETRAIN_LEFT_2, 0, MAX_PWM);
      pwm.setPWM(DRIVETRAIN_RIGHT_1, 0, 0);
      pwm.setPWM(DRIVETRAIN_RIGHT_2, 0, MAX_PWM);

      pwm.setPin(LINEAR_LEFT_1, 0);
      pwm.setPin(LINEAR_LEFT_2, LIFT_PWM);
      pwm.setPin(LINEAR_RIGHT_1, 0);
      pwm.setPin(LINEAR_RIGHT_2, LIFT_PWM);
    } else {
      climbing = false;
      pwm.setPin(DRIVETRAIN_LEFT_1, 0);
      pwm.setPin(DRIVETRAIN_LEFT_2, 0);
      pwm.setPin(DRIVETRAIN_RIGHT_1, 0);
      pwm.setPin(DRIVETRAIN_RIGHT_2, 0);
      
      pwm.setPin(LINEAR_LEFT_1, 4095);
      pwm.setPin(LINEAR_LEFT_2, 4095);
      pwm.setPin(LINEAR_RIGHT_1, 4095);
      pwm.setPin(LINEAR_RIGHT_2, 4095);
    }
  }

  //AUTO BLOCK
  bool currentLimitSwitchState = digitalRead(LIMIT_SWITCH_PIN);

  if (prevLimitSwitchState == HIGH && currentLimitSwitchState == LOW) {
    int pulse = map(angleUp, 0, 180, SERVOMIN, SERVOMAX);  
    pwm.setPWM(SERVO_UP, 0, pulse);
  }
  prevLimitSwitchState = currentLimitSwitchState;  

  delay(15);
}
