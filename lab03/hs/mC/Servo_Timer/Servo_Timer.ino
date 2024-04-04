// Make sure these libraries are installed
// If not the case, use Tools > Manage Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// declaring variables

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servo_num = 0;
int servo_freq = 50;

int pos, MinPulse, MaxPulse;  // current position, minimum (0°) & maximum (180°) pulse length of the servo in µs             
float secondStep;             // unrounded pulse length increase in µs that corresponds to an increase of 1 second
float exactPos;               // Exact, unrounded position of the servo in µs
char serialVariable;          // Character received through the serial communication

int cur_time = 0;


void setup() {
  Serial.begin(115200);                 // open serial communication
  //_________________Begin - Setup_______________
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  MinPulse = 550;
  MaxPulse = 2400;
  //_________________End - Setup_______________

}

void loop() {
  
  // Only run the loop if the serial communication is available

  if( Serial.available())
  {
    
    // read the incoming character and save it in "serialVariable"
    serialVariable = Serial.read();

    // If "a" is received reset the servo to its starting position.
    if (serialVariable == 'a') {
      //_________________Begin - Reset_______________
      pwm.writeMicroseconds(servo_num, MinPulse);
      cur_time = 0;

      Serial.print("timer[s]: ");
      Serial.print(cur_time);
      Serial.print(", pulse[us]: ");
      Serial.println(exactPos);
      //_________________End - Reset_______________
    }
    // If "b" is received move the motor by one step.
    else if (serialVariable == 'b') {
      //_________________Begin - Advance_______________
      cur_time += 1;
      int exactPos = map(cur_time, 0, 150, 550, 2276);
      pwm.writeMicroseconds(servo_num, exactPos);

      Serial.print("timer[s]: ");
      Serial.print(cur_time);
      Serial.print(", pulse[us]: ");
      Serial.println(exactPos);
      //_________________End - Advance_______________

    }

    else if (serialVariable == 'c') {
      //_________________Begin - Advance_______________
      cur_time += 10;
      int exactPos = map(cur_time, 0, 150, 550, 2276);
      pwm.writeMicroseconds(servo_num, exactPos);

      Serial.print("timer[s]: ");
      Serial.print(cur_time);
      Serial.print(", pulse[us]: ");
      Serial.println(exactPos);
      //_________________End - Advance_______________

    }
    else {

    }
    
  }
}
