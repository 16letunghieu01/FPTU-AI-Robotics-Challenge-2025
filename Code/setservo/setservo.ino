#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Cài đặt giá trị xung cho servo
#define SERVOMIN 100   // Giá trị xung tối thiểu (tương ứng 0 độ)
#define SERVOMAX 500   // Giá trị xung tối đa (tương ứng 180 độ)

// Chuyển góc (0–180) thành giá trị PWM (0–4095)
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // Tần số cho servo

  delay(1000); // Chờ ổn định

  // Set servo channel 4 và 5 về 0 độ
  pwm.setPWM(4, 0, angleToPulse(0));
  pwm.setPWM(5, 0, angleToPulse(0));

  Serial.println("Đã set channel 4 và 5 về góc 0 độ.");
}

void loop() {
  // Không làm gì trong loop
}
