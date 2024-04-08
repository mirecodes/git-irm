// Make sure these libraries are installed
// If not the case, use Tools > Manage Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// declare pins
int POTI = A0;

// declare variables
int servo_num = 0;
int servo_freq = 50;

// create a pwm object to control the servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  
  // _______________ Begin - Setup _______________

  // Begin the serial communication
  Serial.begin(115200);
  // Begin PWM communication and set servo frequency
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  // _______________ End - Setup _______________
  
}

void loop() {
  // _______________ Begin - Loop _______________

  // Read values from the analog pin and map/scale them to the movment range of the servo.
  int potential = analogRead(POTI);

  int pwm_microsec = map(potential, 0, 4095, 550, 2276);

  Serial.print("poential: ");
  Serial.print(potential);
  Serial.print(", microsec: ");
  Serial.println(pwm_microsec);

  pwm.writeMicroseconds(servo_num, pwm_microsec);

  // Optionally display the reading from the potentiometer on the serial monitor
  // Set the servo position according to the mapped/scaled value
  
  // _______________ End - Loop _______________
  
  delay(200); // wait for the servo to get there
}
