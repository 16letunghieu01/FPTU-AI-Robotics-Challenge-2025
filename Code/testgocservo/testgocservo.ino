#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_CHANNEL 4 // Thay bằng channel bạn đang thử (4 hoặc 7)
#define STEP 1        // Mỗi lần tăng xung bao nhiêu
#define DELAY_TIME 1000  // Thời gian delay giữa các bước (ms)

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60); // 60Hz phù hợp cho servo
  Wire.setClock(400000);

  Serial.println("Bắt đầu dò vị trí dừng servo...");
}

void loop() {
  for (int pulse = 110; pulse <= 400; pulse += STEP) {
    pwm.setPWM(SERVO_CHANNEL, 0, pulse);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    delay(DELAY_TIME);
  }

  Serial.println("Kết thúc dò. Servo đang ở xung cuối.");
  while (1); // Dừng lại
}
