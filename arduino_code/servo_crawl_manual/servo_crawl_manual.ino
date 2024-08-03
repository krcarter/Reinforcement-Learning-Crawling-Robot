#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Setting up positioning for servos
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  505 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // MF95 update rate of 50

// Update frequency
unsigned long previousMillis = 0;
const int updateInterval = 10; // Update interval in milliseconds

const int numServos = 4; // Number of servos

const float pi = 3.14159265358979323846;
const float frequency = 1.0; // Frequency in Hz

float amplitude[numServos] =     { 15.0, 30.0  , 15.0, 30.0}; // Amplitude in degrees
float phaseShift[numServos] =    {    0, pi/2.0,    0, pi/2.0}; // Phase shift in radians
float verticalShift[numServos] = {-15.0, 75.0  , 15.0, 75.0}; // Vertical shift in degrees

float simulationShift = 90.0;

void setup() {
  Serial.begin(9600);
  Serial.println("Simple Gait Trajectory");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // HACKY but wanted get this up and running quick
  // Set Back legs intials conditions

  int J4 = map(5, 0, 180, SERVOMIN, SERVOMAX);
  int J5 = map(5, 0, 180, SERVOMIN, SERVOMAX);
  int J6 = map(175, 0, 180, SERVOMIN, SERVOMAX);
  int J7 = map(175, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(4, 0, J4);
  pwm.setPWM(5, 0, J5);
  pwm.setPWM(6, 0, J6);
  pwm.setPWM(7, 0, J7);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;
    float currentTime = currentMillis;

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


    // Then, set the pulse lengths for all servos
    for (int i = 0; i < numServos; i++) {
      Serial.print("Index: ");
      Serial.print(i);
      Serial.println(pulseLengths[i]);
      pwm.setPWM(i, 0, pulseLengths[i]);
    }

  }
}
