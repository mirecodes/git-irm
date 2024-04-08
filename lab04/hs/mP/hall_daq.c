#include "feather_serial.h"

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

// Include your includes here

int main()
{
  // Initialize all parameters
  char string_buffer[100] = {};

  // Initialize the serial port on the port /dev/ttyUSB0, with baud rate 115200
  int fd = serialport_init("/dev/cu.usbserial-021FF09A", 115200);

  // Write to the serial port to get a value (or to move the magnet according to the .ino file)
  int res = serialport_writebyte(fd, "c");
      if (res == -1) printf("Unable to write the port \n");

  // Let the user know if you were able to write to the port

  // Read the sensor value from the serial port into the buffer. Give the system enough time to wait for an answer from the serial port with the "timeout" variable. 
  serialport_read(fd, string_buffer, 100, 2);
  int hall_value = atoi(string_buffer);

  // Let the user know if you were able to read from the port
  printf("buffer context: %d\n", hall_value);

  // Convert the sensor value to a voltage
  float voltage = hall_value / 4095.0 * 5;

  // Convert the voltage value to magnetic field

  // Print the magnetic field (and the position) to the terminal (or .txt file)

  // Close the serial port
	serialport_close(fd);

  return 0;
}
