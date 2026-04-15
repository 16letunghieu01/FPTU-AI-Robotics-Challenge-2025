#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Giới hạn pulse an toàn cho servo
#define SERVOMIN 110     // Xung ngắn nhất (quay ngược tối đa)
#define SERVOMAX 600     // Xung dài nhất (quay thuận tối đa)
#define SERVO_STOP 375   // Xung giữa (dừng servo)

#define CHANNEL6 6       // Servo 6
#define CHANNEL7 7       // Servo 7

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);  // Tần số servo 50–60Hz
}

void loop() {
  // Cả hai quay về hai chiều ngược nhau
  pwm.setPWM(CHANNEL6, 0, SERVOMIN); // Quay ngược tối đa
  pwm.setPWM(CHANNEL7, 0, SERVOMAX); // Quay thuận tối đa
  delay(1000);

}
