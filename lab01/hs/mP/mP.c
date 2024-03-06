#include <stdio.h>
#include <stdint.h>
#include "feather_serial.h"

// ***** print_bits() can be implemented here ***** //

// *********************************************** //

int32_t main()
{
  int n;
  // initialization of serial communication, port and baud rate are specified
  int fd = serialport_init("/dev/ttyUSB0", 115200);
  uint8_t arg = (uint8_t)245;

  while (1)
  {
    // ***** your code goes here ***** //

    // ******************************* //

    // send arg via serial communication to the mC
    // type casting is again needed to match type
    n = serialport_writebyte(fd, ((char *)&arg));
    if (n == -1)
      printf("Unable to write the port \n");
  }

  // close serial communication
  serialport_close(fd);
  return 0;
}
