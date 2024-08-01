/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  505 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Digital servos run at ~100 Hz updates

#define servoChannel 0

// our servo # counter
uint8_t servonum = 0;

const int increment = 10; // Degree increment

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

}

void loop() {
  int J4 = map(10, 0, 180, SERVOMIN, SERVOMAX);
  int J5 = map(10, 0, 180, SERVOMIN, SERVOMAX);
  int J6 = map(175, 0, 180, SERVOMIN, SERVOMAX);
  int J7 = map(175, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(4, 0, J4);
  pwm.setPWM(5, 0, J5);
  pwm.setPWM(6, 0, J6);
  pwm.setPWM(7, 0, J7);
}
