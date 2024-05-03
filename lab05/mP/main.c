#include <stdio.h>
#include "util.h"

int main(){
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

  //Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
  const char* port= "/dev/cu.usbserial-01E4BC76";//vm: "/dev/ttyUSB0", mac: "/dev/cu.SLAB_USBtoUART"
  int fd = serialport_init(port, 115200);
  if (fd == -1){
      printf("Could not open the port.\n");
      printf(" - Is the Arduino IDE terminal opened?\n");
      printf(" - Is the device connected to the VM?\n");
      return -1;    
  }

  // Initialize robot and check
  // if messages are received
  initBallBalancingRobot(fd);

  // Make sure that serial port is relaxed
  usleep(20*1000);

  // Parameter loading functions
  load_parameters();
  load_servo();

  //////////////////////////////
  //////// Task Selection //////
  //////////////////////////////
  int task_selection = 0;
  printf("Select Task: ");
  scanf("%d", &task_selection);

  //////////////////////////////
  /////////// Task 1 ///////////
  //////////////////////////////

  if(task_selection == 1){
    /* Test inverse kinematics via
    terminal */
      
    //initalize variables:
    double plate_angles[] = {0,0};
    double servo_angles[] = {0,0,0};
     
    /* ********************* */
    /* Insert your Code here */
    /* ********************* */

    int request = 1;
    while (request) {
      // #1 Get the angles from user
      printf("Input your angles in degree [phy_x] [theta_y]: ");
      scanf("%lf %lf", &plate_angles[0], &plate_angles[1]);

      // #2 Compute the servo angles
      int res = inverseKinematics(plate_angles, servo_angles);

      // #3 Send the command to the servos
      if (res==-1) { // servo angles are not feasible
        printf("Servo angles are not feasible\n");
      }
      else {         // servo angles are feasible
        servoCommand(fd, servo_angles);
      }

      // #4 Print the angles
      printf("Plate angles: phi_x=%.2f, theta_y=%.2f\n", plate_angles[0], plate_angles[1]);
      printf("Servo angles: a_A=%.2f, a_B=%.2f, a_C=%.2f\n", servo_angles[0], servo_angles[1], servo_angles[2]);

      // #5 Ask user whether the process repeats
      int input = 0;
      int valid = 0;

      while (valid==0) {
        printf("Do you want to repeat? [1]yes, [2]no: ");
        scanf("%d", &input);

        if (input==1) {
          request = 1;
          valid = 1;
        }

        else if (input==2) {
          request = 0;
          valid = 1;
        }

        else {
          printf("Input the right values\n");
          valid = 0;
        }
      }

    }

  }

  //////////////////////////////
  /////////// Task 2 ///////////
  //////////////////////////////
  /*Test projection from the image frame to the world frame*/
  if(task_selection == 2){
      
      //initalize variables:
      
    /* ********************* */
    /* Insert your Code here */
    /* ********************* */

   
  }


  return 0;
}
