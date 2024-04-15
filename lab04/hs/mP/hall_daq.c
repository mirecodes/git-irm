#include "feather_serial.h"

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

// Include your includes here

int main(int argc, const char * argv[])
{
  // Initialize all parameters
  char string_buffer[100] = {};
  int res = 0;
  char wb = 'a';
  char str[2] = {};

  // Argument Check
  if (argc == 2) {
    if (argv[1][0] == 'a') wb = 'a';
    else if (argv[1][0] == 'b') wb = 'b';
    else if (argv[1][0] == 'c') wb = 'c';
  }

  // Initialize the serial port on the port /dev/ttyUSB0, with baud rate 115200
  int fd = serialport_init("/dev/cu.usbserial-021FF09A", 115200);

  for (int i=0; i<50; i++) {
    // Write to the serial port to get a value (or to move the magnet according to the .ino file) 
    str[0] = wb;
    str[1] = '\0';
    res = serialport_writebyte(fd, str);

    // Let the user know if you were able to write to the port
    if (res == -1) printf("#[Error] Unable to write the port \n");

    // Read the sensor value from the serial port into the buffer. Give the system enough time to wait for an answer from the serial port with the "timeout" variable. 
    res = serialport_read(fd, string_buffer, 100, 50);

    // Let the user know if you were able to read from the port
    if (res < 0) printf("#[Error] Unable to read the port \n");

    // Convert the sensor value to a voltage
    int hall_value = atoi(string_buffer);
    float voltage = hall_value / 4095.0 * 3.3;
    printf("%.3f\n", voltage);
    // Convert the voltage value to magnetic field

    // Print the magnetic field (and the position) to the terminal (or .txt file)
  }

  // Close the serial port
	serialport_close(fd);

  return 0;
}
