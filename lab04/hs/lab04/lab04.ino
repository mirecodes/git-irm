// declare pins
int INPUT_PIN = A2;

void setup() {
  // _______________ Begin - Setup _______________
  // Begin the serial communication
  Serial.begin(115200);
  // _______________ End - Setup _______________

}

void loop() {
  // _______________ Begin - Loop _______________
  if( Serial.available())
  {
    // read the incoming character and save it in "serialVariable"
    char serial_variable = Serial.read();

    // If "q" is received, quit the loop
    if (serial_variable == 'q') {
      Serial.println("Terminate the process");
      return;
    }
    else if (serial_variable == 'c') {
      int input = analogRead(INPUT_PIN);
      // analogRead: for ArduinoUNO, from 0 to 1023 (0-5 V)
      // analogRead: for ESP32 Feather, from 0 to 4095 (0-3.3 V)
      float voltage = input/1024.0 * 5;
      Serial.print("input_voltage[V] = ");
      Serial.println(voltage);
    }
  }
  // _______________ End - Loop _______________
}
