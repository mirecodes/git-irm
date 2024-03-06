#include <stdio.h>
#include <stdint.h>
#include "feather_serial.h"

// ***** print_bits() can be implemented here ***** //
void print_bits(uint8_t arg_word)
{
	printf("hex: 0x%02x, bin: ", arg_word);

	// only a simple implementation of a for loop is given
	// you are free to use it or solve the problem in another way

	for (int i=1; i>-1; i--) {
		char bin_digits[4] = {};

		bin_digits[0] = 0 != (arg_word & 1<<(3+i*4));
		bin_digits[1] = 0 != (arg_word & 1<<(2+i*4));
		bin_digits[2] = 0 != (arg_word & 1<<(1+i*4));
		bin_digits[3] = 0 != (arg_word & 1<<(0+i*4));

		for (int j=0; j<4; j++) {
			printf("%d", bin_digits[j]);
		}
		printf(" ");
	}
	printf("\n");
}
// *********************************************** //

int32_t main()
{
  int n;
  // initialization of serial communication, port and baud rate are specified
  int fd = serialport_init("/dev/cu.usbserial-022F296B", 115200);
  uint8_t arg = (uint8_t)245;

  while (1)
  {
    // ***** your code goes here ***** //
    int a, b = 0; 
    scanf("%d %d", &a, &b);

    uint8_t arg = (a+b) & ((1<<8) - 1);

    print_bits(arg);
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
