/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->  http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h> // Include Adafruit Motor Shield Library for cmmands
#define interruptPin A0   // Pin that reads whether the crash button is pressed
#define hall_pin A1       // Pin that reads the analog hall sensor voltage

char incoming;                // Store the incoming character from the serial port

uint16_t incoming_Hall;   // Store the hall sensor voltage
uint32_t sum;             // Store the sum of the sensor voltages
uint16_t hall_Mean;       // Store the average of the sensor voltages

float position = 0.0;       // Magnet position in mm
float position_calib = 15.0;  // Starting postition after calibration (Change if needed)

// Interrupt variables
unsigned long last_pressed = 0;   // Store time when button has last been pressed
unsigned int debounceTime = 100;  // in ms, Time to avoid button debouncing
bool interrupt_Flag = false;  // Interrupt Flag to store button state
bool interrupt_Read = false;  // Digital Read to store button state at beginning of loop
bool read_Pos = true;         // Determine if you digitally want to read the position. Set to fals if you don't

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);


void setup() {

  Serial.begin(115200);  // set up Serial library at 9600 bps

  attachInterrupt(digitalPinToInterrupt(interruptPin), crash_Interrupt, FALLING); // Define interrupt at correct pin, calling the "crash" function in FALLING mode

  // Connect to Motor wing and to the stepper motor
  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
                        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }
  Serial.println("Motor Shield found.");

  myMotor->setSpeed(30);  // 30 rpm

}

void loop() {
  
  // (DO NOT CHANGE THIS Code) Check the current state of the interrupt pin or reset it if it's still on
  bool interrupt_Read = digitalRead(interruptPin);
  if (interrupt_Read == 1)
  {
    interrupt_Flag = false;
  }
  else
  {
    interrupt_Flag = true;
  }
  // ------------------------------------------------------- 

  //-------------------------
  //----- Prelab 2 ----------
  //-------------------------

  // Write your code here

  //-------------------------




  //-------------------------
  //----- Postlab 2 ----------
  //-------------------------

  // Write your code snippet here to read in 20 consecutive values

  //-------------------------



  //-------------------------
  //----- Postlab 3 ----------
  //-------------------------

  // Write your code here to call the motor movement functions depending on the input

  //-------------------------

}

// --------------------------------------------------------------------------------------
// ------- Functions to move the stepper motor and prevent the stage from crashing ------
// --------------------------------------------------------------------------------------

// ---- Calibrate (DO NOT CHANGE THIS FUNCTION) ----
// This function will move the stage until it reaches the safety switch, trigger the crash() function and then move back to the starting position of 20mm

void calibrate(){
  //Serial.println("Calibration");
  while(!interrupt_Flag){                 // As long as the safety switch is not pressed (the interrupt flag is not TRUE) move the magnet forward
    myMotor->step(1, FORWARD, DOUBLE);
  }
  myMotor->step(100, BACKWARD, DOUBLE);   // After the button is detected, move 200 steps back (2mm)
  myMotor->release();                     // Release the motor after moving so it does not use current and heat up
  position = position_calib;              // Update the postition to the starting position. Position_calib may vary depending on the setup, feel free to change it to its correct value
  Serial.print(1);                        // Send back the character "1" to let the C-Code know, that the calibration is complete
}
       


// ---------------------------------------------------
// ----------------- Postlab Q3 ----------------------
// ---------------------------------------------------

//        ----- Write your function(s) here -----

// ---------------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------




// ---- Crash Interrupt (DO NOT CHANGE THIS FUNCTION) ----
// This is the interrupt function. If the safety button is pressed it will set interrupt_Flag to TRUE. 
// Since the button takes a little time to bounce back when released a debounce time is added to prevent it from firing multiple times when triggered once.
// Use IF or WHILE statements with interrupt_Flag to prevent the motor from crashing the magnet stage when it triggers the button

void crash_Interrupt(){

  if ((millis() - last_pressed) > debounceTime) 
  {
    interrupt_Flag = true;
    // Serial.println("Switch pressed!");
    last_pressed = millis();
  }
}