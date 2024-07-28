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

#define servoChannel 0

// our servo # counter
// uint8_t servonum = 0;

const int numServos = 8; // Number of servos

const float pi = 3.14159265358979323846;
const float frequency = 1.0; // Frequency in Hz
const int updateInterval = 20; // Update interval in milliseconds

float amplitude[numServos] = {15.0, 30.0, 0, 0, 0, 0, 0, 0}; // Amplitude in degrees
float phaseShift[numServos] ={0 ,  pi/2.0,  0,  0, 0, 0, 0, 0}; // Phase shift in radians
float verticalShift[numServos] = {-15.0, 75.0, 0, 0, 0, 0, 0, 0}; // Vertical shift in degrees

float simulationShift = 90.0;

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
  unsigned long currentTime = millis();

  //float amplitude = 30.0;
  //float angle = amplitude * sin(2 * pi * frequency * currentTime / 1000.0) + 90.0; // Sine wave centered at 90 degrees
  //int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  //pwm.setPWM(servoChannel, 0, pulseLength);

  // First, calculate all the angles and pulse lengths
  int pulseLengths[numServos];
  for (int i = 0; i < numServos; i++) {
    float angle = 0.0;
    if (i % 2 == 0) {
      // aligning the -90 to 90 from simulation to 0 to 180 from the robot
      float flipVerticalShift = -1.0; 
      float angle = amplitude[i] * sin(2 * PI * frequency * currentTime / 1000.0 + phaseShift[i]) + (flipVerticalShift * verticalShift[i]);
      angle += 90.0;  // Add 90 degrees to the angle for servos at odd indices this aligns to axis on the simulation
      // Serial.print(("Index: "));
      // Serial.println(i);
      pulseLengths[i] = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    }
    else{
      float angle = amplitude[i] * sin(2 * PI * frequency * currentTime / 1000.0 + phaseShift[i]) + verticalShift[i]+30.0;
      pulseLengths[i] = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    }
  }

  // Set Back legs intials conditions

  pulseLengths[4] = map(15, 0, 180, SERVOMIN, SERVOMAX);
  pulseLengths[5] = map(20, 0, 180, SERVOMIN, SERVOMAX);
  pulseLengths[6] = map(150, 0, 180, SERVOMIN, SERVOMAX);
  pulseLengths[7] = map(160, 0, 180, SERVOMIN, SERVOMAX);


  // Then, set the pulse lengths for all servos
  for (int i = 0; i < numServos; i++) {
    //Serial.println(pulseLengths[i]);
    pwm.setPWM(i, 0, pulseLengths[i]);
  }

  //delay(updateInterval); // Wait for next update
}
