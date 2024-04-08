#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

// Include your includes here

int main()
{
  // Initialize all parameters

  // Initialize the serial port on the port /dev/ttyUSB0, with baud rate 115200

  // Write to the serial port to get a value (or to move the magnet according to the .ino file)

  // Let the user know if you were able to write to the port

  // Read the sensor value from the serial port into the buffer. Give the system enough time to wait for an answer from the serial port with the "timeout" variable. 

  // Let the user know if you were able to read from the port

  // Convert the sensor value to a voltage

  // Convert the voltage value to magnetic field

  // Print the magnetic field (and the position) to the terminal (or .txt file)

  // Close the serial port

  return 0;
}
