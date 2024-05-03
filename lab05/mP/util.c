#include "util.h"
#include "newton_raphson.h"

int initBallBalancingRobot(int fd) {
  
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#######################\n");
  printf("Hardware Initialization\n");
  printf("#######################\n");

  // Let feather reboot
  usleep(200);

  // Check pixy readings
  int pixy_return, flag, x, y;
  pixy_return = readFromPixy(fd, &flag, &x, &y);
  while (!pixy_return) {
    // retry until connection established
    pixy_return = readFromPixy(fd, &flag, &x, &y);
  }
  
  printf("%s, x = %d, y = %d \n", "INIT: Pixy Coordinates received", x, y);

  // Do some inverse kinematics to check for errors
  double position[] = {0, 0, 130};
  double plate_angles[] = {0, 0};
  double servo_angles[] = {0, 0, 0};
  inverseKinematics(plate_angles, servo_angles);
  printf("INIT: Inverse Kinematics initialized\n");
  printf("INIT: Finished\n");
  // printf("INIT: A: %.2f, B: %.2f, C: %.2f
  // \n",servo_angles[0],servo_angles[1],servo_angles[2]);

  tcflush(fd, TCIFLUSH);

  // Send motor commands
  // servoCommand(fd,servo_angles);

  return 1;
}

int inverseKinematics(const double* plate_angles, double* servo_angles) {
  // Load parameters R, L_1, L_2, P_z etc. from parameters file. Example: double R = bbs.R_plate_joint;
  // Then implement inverse kinematics similar to prelab
  
  load_parameters();
  double R = bbs.R_plate_joint;
  double L_1 = bbs.l1;
  double L_2 = bbs.l2;
  double P_z = bbs.plate_height;

  double phi_x = plate_angles[0] * M_PI / 180;
  double theta_y = plate_angles[1] * M_PI / 180;

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */
  
  // Calculate delta_z values
  double delta_zA = R * sin(phi_x);
  double delta_zB = (sqrt(3)/2*sin(theta_y) - 1/2*sin(phi_x))*R;
  double delta_zC = (-sqrt(3)/2*sin(theta_y) - 1/2*sin(phi_x))*R;
  
  servo_angles[0] = calculateAlpha(L_1, L_2, delta_zA, P_z) * 180 / M_PI;
  servo_angles[1] = calculateAlpha(L_1, L_2, delta_zB, P_z) * 180 / M_PI;
  servo_angles[2] = calculateAlpha(L_1, L_2, delta_zC, P_z) * 180 / M_PI;

  // return -1; // if invalid input angle
  // we checked beta > -pi/2 (cannot be folded to perpendicular position with inversed direction)
  if (servo_angles[0] < 0 || servo_angles[1] < 0 || servo_angles[2] < 0) return -1;
  // if ok
  return 0; 
};

double calculateAlpha(double L_1, double L_2, double delta_z, double P_z) {
  double numerator = pow(L_2, 2)- (pow(delta_z+P_z, 2) + pow(L_1, 2));
  double denominator = 2*(delta_z + P_z)*L_1;
  double beta = acos(numerator/denominator);
  double alpha = M_PI_2 - beta;
  return alpha;
}




int project2worldFrame(const int x_in, const int y_in, double* x_out, double* y_out){

    // implement the code to project the coordinates in the image frame to the world frame
    // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
    // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.
    
    /* ********************* */
    /* Insert your Code here */
    /* ********************* */
    
  return 0;
};




/* Sends servo angles to serial port */
int servoCommand(int fd, double* servo_angles) {
  // check serial
  int writeval;

  // assign values
  double angleA = servo_angles[0] + servo.bias_A;
  double angleB = servo_angles[1] + servo.bias_B;
  double angleC = servo_angles[2] + servo.bias_C;

  int min = servo.min_angle;
  int max = servo.max_angle;

  // check if values are valid
  int condition = (angleA < max && angleA > min) &&
                  (angleB < max && angleB > min) &&
                  (angleC < max && angleC > min);

  if (condition != 1) {
    printf("ERROR: Servo angles out of bounds.\n");
    return -1;
  }

  // assemble command
  char command[50];
  sprintf(command, "C %.2f %.2f %.2f\n", angleA, angleB, angleC);

  // Flush serial port output
  tcflush(fd, TCOFLUSH);
  // send command
  writeval = write(fd, command, strlen(command));

  return 0;
}

/* Reads pixel coordinates from Pixycam. Also returns a flag whether an object
 * was detected or not */
int readFromPixy(int fd, int* flag, int* x, int* y) {
  char buff[20];
  const char command[] = "P\n";
  int writeval;
  char* token;
  const char delim[] = " ";

  // Flush serial port input
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);

  // Write command to pixy
  writeval = serialport_write(fd, command);
  usleep(10 * 1000);

  // Read until until no more bytes available
  // If not data is availabe, retry until success
  int readval = 0;
  while(readval != 1){
    readval = serialport_read_until(fd, buff, sizeof(buff), '\n', 100);
  }
  
  // printf("readFromPixy: after read \n");
  // printf("writeval = %d, readval = %d", writeval,readval);

  // Catch read write errors
  if (!readval) {
    //printf("SERIAl READ FAILED with %d \n",readval);
    return -1;
  }

  // Add terminating 0 to make string
  buff[sizeof(buff) - 1] = 0;

  // extract values using strtok
  token = strtok(buff, delim);

  // Verify initial character
  if (token[0] != 'A') {
    //printf("SERIAL HEADER ERROR: %.20s\n",buff);
    return -1;
  }

  token = strtok(NULL, delim);
  *flag = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *x = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *y = atoi(token);

  return 1;
}

