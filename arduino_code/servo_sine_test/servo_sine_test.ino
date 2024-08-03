/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  505 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Digital servos run at ~100 Hz updates
unsigned long previousMillis = 0;
const int updateInterval = 20; // Update interval in milliseconds

#define servoChannel 0

// our servo # counter
uint8_t servonum = 0;

const float pi = 3.14159265358979323846;
const float frequency = 2.0; // Frequency in Hz

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    float currentTime = currentMillis;

    float amplitude = 10.0;
    float angle = amplitude * sin(2 * pi * frequency * currentTime / 1000.0) + 90.0; // Sine wave centered at 90 degrees
    int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);

    //pwm.setPWM(servoChannel, 0, pulseLength);
    pwm.setPWM(servoChannel, 0, pulseLength);
    pwm.setPWM(2, 0, pulseLength);
    pwm.setPWM(4, 0, pulseLength);
    pwm.setPWM(6, 0, pulseLength);
      
    // Your servo command code here
  }
}
