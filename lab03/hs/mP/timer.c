#include "feather_serial.h"

/* These are system level includes */
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

void delay(int number_of_seconds)
{
    int micro_seconds = 1000000 * number_of_seconds;
  
    clock_t start_time = clock();
  
    while (clock() < start_time + micro_seconds);
}


int main()
{   // Intialize serial port and variables
    int fd = serialport_init("/dev/cu.usbserial-021FF09A", 115200);
    
    int correct_input_value = 0; // "boolean" value that switches to 1 when a suitable input has been received
    int run_time;                // Amount of seconds the timer is supposed to run
    int i;                       // Iteration variable

    char* reset = "a";           // Character to be sent through the serial port to reset the servo to its 0° position.
    char* increase = "b";        // Character to be sent through the serial port to tell the Arduino to advance by 1 second.
    
    // loop until the user wants to quit the timer program
    while (1) {
        /*_________________Begin - Input Section_________________*/
        printf("Give time to run (0-150, 0: stop): ");
        scanf("%d", &run_time);

        if (run_time > 0 && run_time < 151) {
            correct_input_value = 1;
        }   
        else if (run_time == 0) {
            int res = serialport_writebyte(fd, reset);
            if (res == -1) printf("Unable to write the port \n");
            break;
        }
        else {
            printf("Use a different value\n");
        }
        
        /*_________________End - Input Section_________________*/

        
        /*_________________Begin - Reset Section_________________*/
        int res = serialport_writebyte(fd, reset);
        if (res == -1) printf("Unable to write the port \n");

        /*_________________End - Reset Section_________________*/
        
		delay(1); // Wait a second before starting the timer
        printf("%d seconds will be timed, starting now! \n", run_time);
        
        /*_________________Begin - Advance Section_________________*/
        for (int i=0; i<run_time; i++) {
            int res = serialport_writebyte(fd, increase);

            if (res == -1) printf("Unable to write the port \n");
            char buff[100] = {};
            serialport_read(fd, buff, 100, 2);
            printf("%s", buff);

            delay(1);  
        }
        /*_________________End - Advance Section_________________*/
    }
    
    // Close the serial port
	serialport_close(fd);
    
	return 0;
    
}
