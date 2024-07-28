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
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  1100 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 100 // Digital servos run at ~100 Hz updates

#define servoChannel 7

// our servo # counter
uint8_t servonum = 0;

const int increment = 10; // Degree increment

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

}

void loop() {
  // Initial position test
  for (int angle = 0; angle <= 180; angle += increment) {
    int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoChannel, 0, pulseLength);

    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print("\tPulse Length: ");
    Serial.println(pulseLength);

    delay(1000); // Wait 1 second between each increment
  }

  // Move back down from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle -= increment) {
    int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoChannel, 0, pulseLength);

    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print("\tPulse Length: ");
    Serial.println(pulseLength);

    delay(1000); // Wait 1 second between each decrement
  }
}
