#include "feather_serial.h"

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

// Include your includes here
#include "hall_sensor.h"


int main(int argc, const char * argv[])
{
  // Initialize all parameters
  char string_buffer[100] = {};
  int res = 0;
  char wb = 'a';
  char str[2] = {};

  int opr_mode = 2;

  // Argument Check
  if (argc == 2) {
    if (argv[1][0] == 'a') wb = 'a';
    else if (argv[1][0] == 'b') wb = 'b';
    else if (argv[1][0] == 'c') wb = 'c';
  }

  // Initialize the serial port on the port /dev/ttyUSB0, with baud rate 115200
  int fd = serialport_init("/dev/cu.usbserial-01D95008", 115200);

  fprintf(stderr, "Type your operation mode (2: task2, 3: task3): ");

  scanf("%d", &opr_mode);

  // select opr mode for task 2
  if (opr_mode == 2) {
    for (int i=0; i<50; i++) {
    // Write to the serial port to get a value (or to move the magnet according to the .ino file) 
    str[0] = wb;
    str[1] = '\0';
    res = serialport_writebyte(fd, str);

    // Let the user know if you were able to write to the port
    if (res == -1) printf("[Error] Unable to write the port \n");

    // Read the sensor value from the serial port into the buffer. Give the system enough time to wait for an answer from the serial port with the "timeout" variable. 
    res = serialport_read(fd, string_buffer, 100, 50);

    // Let the user know if you were able to read from the port
    if (res < 0) printf("[Error] Unable to read the port \n");

    // Convert the sensor value to a voltage
    int hall_value = atoi(string_buffer);
    float voltage = hall_value / 4095.0 * 3.3;
    printf("%.3f\n", voltage);
    // Convert the voltage value to magnetic field

    // Print the magnetic field (and the position) to the terminal (or .txt file)
    }
  }

  // select opr mode for task 3
  else if (opr_mode == 3) {
    // Initialize Variation
    int start_pos = 18;
    int cur_pos = start_pos;
    int lim_pos = 56;

    float voltage_q = 1.529;
    float B = 0.0;

    float mC_pos = 0.0;

    int output = 0;

    // Calibration
    res = serialport_writebyte(fd, "0");
    // Let the user know if you were able to write to the port
    if (res == -1) printf("[Error] Unable to write the port \n");

    // Wait until mC complete the calibration
    do {
      res = serialport_read(fd, string_buffer, 100, 10);
      output = atoi(string_buffer);
    } while (output != 1);

    // Start movement
    while (cur_pos < lim_pos) {

      // Write port to get the voltage value
      res = serialport_writebyte(fd, "b");
      // Let the user know if you were able to write to the port
      if (res == -1) printf("#[Error] Unable to write the port \n");
      // Read the voltage value
      res = serialport_read(fd, string_buffer, 100, 50);
      // Let the user know if you were able to read from the port
      if (res < 0) printf("#[Error] Unable to read the port \n");
      // Convert the sensor value to a voltage
      int hall_value = atoi(string_buffer);
      float voltage = hall_value / 4095.0 * 3.3;

      // Write port to get the position value
      res = serialport_writebyte(fd, "p");
      // Let the user know if you were able to write to the port
      if (res == -1) printf("#[Error] Unable to write the port \n");
      // Read the position value
      res = serialport_read(fd, string_buffer, 100, 50);
      // Let the user know if you were able to read from the port
      if (res < 0) printf("#[Error] Unable to read the port \n");
      // Get the position value
      mC_pos = atof(string_buffer);

      // Convert the voltage value to magnetic field
      B = hall_sensor_get_field(voltage, voltage_q);

      // Print the magnetic field (and the position) to the terminal (or .txt file)
      fprintf(stderr, "(distance [mm]) - (Magnetic Field [mT]): %f - %f\n", mC_pos, B);
      fprintf(stdout, "%f - %f\n", mC_pos, B);

      // Write mC to move the position
      res = serialport_writebyte(fd, "1");
      // Let the user know if you were able to write to the port
      if (res == -1) printf("[Error] Unable to write the port \n");

      // Wait until mC complete the movement
      do {
        res = serialport_read(fd, string_buffer, 100, 10);
        output = atoi(string_buffer);
      } while (output != 2);

      cur_pos += 1;

    }
  }

  // Close the serial port
	serialport_close(fd);

  return 0;
}
