#include "positions.h"  // Include the positions header file
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Setting up positioning for servos
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  505 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // MF95 update rate of 50


unsigned long previousMillis = 0;
const long updateInterval = 40;  // updateInterval in milliseconds

int currentPosition = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Simulation to Real");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // HACKY but wanted get this up and running quick
  // Set Back legs intials conditions

  int J4 = map(90, 0, 180, SERVOMIN, SERVOMAX);
  int J5 = map(90, 0, 180, SERVOMIN, SERVOMAX);
  int J6 = map(90, 0, 180, SERVOMIN, SERVOMAX);
  int J7 = map(90, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(4, 0, J4);
  pwm.setPWM(5, 0, J5);
  pwm.setPWM(6, 0, J6);
  pwm.setPWM(7, 0, J7);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    // Set the servo to the current position
    Serial.print(currentPosition);
    Serial.print(":  ");
    Serial.print(row_1[currentPosition]);
    Serial.print(", ");
    Serial.print(row_2[currentPosition]);
    Serial.print(", ");
    Serial.print(row_3[currentPosition]);
    Serial.print(", ");
    Serial.println(row_4[currentPosition]);


    int J0 = map(row_1[currentPosition], 0, 180, SERVOMIN, SERVOMAX);
    int J1 = map(row_2[currentPosition], 0, 180, SERVOMIN, SERVOMAX);
    int J2 = map(row_3[currentPosition], 0, 180, SERVOMIN, SERVOMAX);
    int J3 = map(row_4[currentPosition], 0, 180, SERVOMIN, SERVOMAX);

    // int J0 = map(90, 0, 180, SERVOMIN, SERVOMAX);
    // int J1 = map(90, 0, 180, SERVOMIN, SERVOMAX);
    // int J2 = map(90, 0, 180, SERVOMIN, SERVOMAX);
    // int J3 = map(90, 0, 180, SERVOMIN, SERVOMAX);

    pwm.setPWM(0, 0, J0);
    pwm.setPWM(1, 0, J1);
    pwm.setPWM(2, 0, J2);
    pwm.setPWM(3, 0, J3);

    // Move to the next position
    currentPosition++;

    // Loop back to the beginning if at the end
    if (currentPosition >= numPositions) {
      currentPosition = 0;
    }
  }
}
