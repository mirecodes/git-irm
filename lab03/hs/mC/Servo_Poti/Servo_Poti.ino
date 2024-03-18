// Make sure these libraries are installed
// If not the case, use Tools > Manage Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// declare variables
int servo_num = 0;
int servo_freq = 50;

// create a pwm object to control the servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  
  // _______________ Begin - Setup _______________

  // Begin the serial communication
  // Begin PWM communication and set servo frequency

  
  // _______________ End - Setup _______________
  
}

void loop() {
  // _______________ Begin - Loop _______________

  // Read values from the analog pin and map/scale them to the movment range of the servo.
  // Optionally display the reading from the potentiometer on the serial monitor
  // Set the servo position according to the mapped/scaled value
  
  // _______________ End - Loop _______________
  
  delay(15); // wait for the servo to get there
}
