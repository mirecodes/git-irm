#include "util.h"
#include "newton_raphson.h"

int inverseKinematics(const double *plate_angles, double *servo_angles)
{
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
  double delta_zB = (sin(theta_y) * sqrt(3) / 2 - 0.5 * sin(phi_x)) * R;
  double delta_zC = (-sin(theta_y) * sqrt(3) / 2 - 0.5 * sin(phi_x)) * R;

  servo_angles[0] = calculateAlpha(L_1, L_2, delta_zA, P_z) * 180 / M_PI + servo.bias_A;
  servo_angles[1] = calculateAlpha(L_1, L_2, delta_zB, P_z) * 180 / M_PI + servo.bias_B;
  servo_angles[2] = calculateAlpha(L_1, L_2, delta_zC, P_z) * 180 / M_PI + servo.bias_C;

  printf("delta_zA: %.2f, servo_angles: %.2f\n", delta_zA, servo_angles[0]);
  printf("delta_zB: %.2f, servo_angles: %.2f\n", delta_zB, servo_angles[1]);
  printf("delta_zC: %.2f, servo_angles: %.2f\n", delta_zC, servo_angles[2]);

  // return -1; // if invalid input angle
  // we checked beta > -pi/2 (cannot be folded to perpendicular position with inversed direction)
  if (servo_angles[0] > 90.0 || servo_angles[1] > 90.0 || servo_angles[2] > 90.0)
    return -1;
  // if ok
  return 0;
};

double calculateAlpha(double L_1, double L_2, double delta_z, double P_z)
{
  double numerator = pow(delta_z + P_z, 2) + pow(L_1, 2) - pow(L_2, 2);
  double denominator = 2 * (delta_z + P_z) * L_1;
  double beta = acos(numerator / denominator);

  double alpha = M_PI_2 - beta;
  return alpha;
}

int cameraCalibration(const int x_in, const int y_in, double *x_out, double *y_out)
{

  // implement the code to project the coordinates in the image frame to the world frame
  // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
  // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */

  // Load Parameters
  load_parameters();
  double u_0 = bbs.distortion_center[0];
  double v_0 = bbs.distortion_center[1];
  double focal_length_in_pixel = bbs.focal_length;
  double k1 = bbs.radial_distortion_coeff[0];
  double k2 = bbs.radial_distortion_coeff[1];
  double image_scale = bbs.calibration_image_scale;

  double cam_offset[3] = {bbs.cam_offset[0], bbs.cam_offset[1], bbs.cam_offset[2]};
  double P_z = bbs.plate_height;
  double ball_radius = bbs.ball_radius;

  // STEP1: Undistort
  // Normalize The Pixels
  double u_norm = (x_in * image_scale - u_0) / focal_length_in_pixel;
  double v_norm = (y_in * image_scale - v_0) / focal_length_in_pixel;

  double r_d = sqrt(pow(u_norm, 2) + pow(v_norm, 2));
  double r = newtonRaphson(r_d, k1, k2);

  double distortion_coeff_r = 1 + k1 * pow(r, 2) + k2 * pow(r, 4);

  double u_undistort = u_norm / distortion_coeff_r;
  double v_undistort = v_norm / distortion_coeff_r;

  // STEP2: Normalized Pixel Frame to Camera Frame
  double z_c = cam_offset[2] + P_z + ball_radius; // assume change of z_c is neglectable
  double x_c = u_undistort * z_c;
  double y_c = v_undistort * z_c;

  // STEP3: Translation into World Frame
  x_out[0] = -x_c + cam_offset[0];
  y_out[0] = -y_c + cam_offset[1];

  return 0;
};

double discreteDerivative(const double dt, const double *x)
{
  // TODO: Implement a discrete derivative function

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */

  double vel = (x[0] - x[1]) / dt;
  return vel;
};

double movingAverage(const int n, const double *x)
{
  // TODO: Implement a moving average function
  /* ********************* */
  /* Insert your Code here */
  /* ********************* */

  double sum = 0;
  for (int i = 0; i < n; i++)
  {
    sum += x[i];
  }

  return sum / n;
};

double butterWorth(const double *x, const double *y)
{
  // TODO: Implement this if you like bonus points (not required to reach max points)
  /* ********************* */
  /* Insert your Code here */
  /* ********************* */
  return 0;
};

int stepResponse(const double current_time, double *x_ref, double *y_ref,
                 double *vx_ref, double *vy_ref)
{

  double step_start = 5;
  double step_distance = 80;
  if (current_time < step_start)
  {
    *x_ref = 0;
  }
  else
  {
    *x_ref = step_distance;
  }

  *y_ref = 0;
  *vx_ref = 0;
  *vy_ref = 0;

  return 0;
};

int circularTrajectory(const double current_time, double *x_ref, double *y_ref,
                       double *vx_ref, double *vy_ref)
{

  double traj_start = 3;
  double num_of_traj = 5;
  double period = 4; // seconds
  double R = 80;     // radius

  // TODO: Implement the circular trajectory function.
  //  Hint: Use the equations for parametrizing a cirlce (and its derivative)

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */
  double t = current_time - traj_start;
  double ang = (t / period) * 2 * M_PI;
  double ang_vel = ang / period;

  if (t > traj_start && t < num_of_traj * period)
  {
    *x_ref = R;
    *y_ref = 0;
    *vx_ref = 0;
    *vx_ref = 0;
  }
  else
  {
    *x_ref = R * sin(ang);
    *y_ref = R * cos(ang);
    *vx_ref = R * ang_vel * (-sin(ang));
    *vy_ref = R * ang_vel * cos(ang);
  }

  return 0;
};

// Don't change any of the below functions.

int initBallBalancingRobot(int fd)
{
  // don't change this function.
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#######################\n");
  printf("Hardware Initialization\n");
  printf("#######################\n");

  // Let feather reboot
  usleep(200);

  // Check pixy readings
  int pixy_return, flag, x, y;
  pixy_return = readFromPixy(fd, &flag, &x, &y);
  while (!pixy_return)
  {
    // retry until connection established
    pixy_return = readFromPixy(fd, &flag, &x, &y);
  }

  printf("%s, x = %d, y = %d \n", "INIT: Pixy Coordinates received", x, y);

  // Do some inverse kinematics
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

/* Sends servo angles to serial port */
int servoCommand(int fd, double *servo_angles)
{
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

  if (condition != 1)
  {
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
  usleep(1200); // wait for write function to complete

  return 0;
}

/* Reads pixel coordinates from Pixycam. Also returns a flag whether an object
 * was detected or not */
int readFromPixy(int fd, int *flag, int *x, int *y)
{
  char buff[20];
  const char command[] = "P\n";
  int writeval;
  char *token;
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
  while (readval != 1)
  {
    readval = serialport_read_until(fd, buff, sizeof(buff), '\n', 100);
  }

  // printf("readFromPixy: after read \n");
  // printf("writeval = %d, readval = %d", writeval,readval);

  // Catch read write errors
  if (!readval)
  {
    // printf("SERIAl READ FAILED with %d \n",readval);
    return -1;
  }

  // Add terminating 0 to make string
  buff[sizeof(buff) - 1] = 0;

  // extract values using strtok
  token = strtok(buff, delim);

  // Verify initial character
  if (token[0] != 'A')
  {
    // printf("SERIAL HEADER ERROR: %.20s\n",buff);
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

long getMicroseconds()
{
  struct timeval the_time;
  long microseconds;
  gettimeofday(&the_time, NULL);
  microseconds = the_time.tv_sec * 1000000 + the_time.tv_usec / 1000 + the_time.tv_usec;
  return microseconds;
}

int pushBack(const double x_new, double *x_array, int array_size)
{
  for (int i = array_size - 1; i > 0; i = i - 1)
  {
    x_array[i] = x_array[i - 1];
  }

  x_array[0] = x_new;
  return 1;
};

int startLogging(FILE *fp, int task_selection, double k_p, double k_d,
                 double k_i, int n_pos, int n_vel)
{
  fprintf(fp, "#Task Selector: %d\n", task_selection);
  fprintf(fp, "#PID Parameters: kp: %.2f, kd: %.2f, ki: %.2f\n", k_p, k_d, k_i);
  fprintf(fp, "#n_pos: %d\n", n_pos);
  fprintf(fp, "#n_vel: %d\n", n_vel);
  fprintf(fp,
          "t x_ref y_ref vx_ref vy_ref x_raw y_raw x y vx_raw vy_raw vx vy\n");
  fclose(fp);

  return 0;
}

int logger(FILE *fp,
           long end_time,
           double current_time,
           double dt,
           double k_p,
           double k_d,
           double k_i,
           double x_ref,
           double y_ref,
           double vx_ref,
           double vy_ref,
           double x_raw,
           double y_raw,
           double x,
           double y,
           double vx_raw,
           double vy_raw,
           double vx,
           double vy,
           double *plate_angles,
           double *servo_angles,
           double x_integrator,
           double y_integrator)
{

  printf("\e[1;1H\e[2J");
  printf("##################################\n");
  printf("############# PID LOOP ###########\n");
  printf("##################################\n");
  printf("Timing\n");
  printf("Elapsed Time: %.2f s\n", current_time);
  printf("Step Time: %.4f s\n", dt);
  printf("Frequency: %.1f Hz\n\n", 1 / dt);
  printf("PID Parameters\n");
  printf("k_p: %.2f, k_d: %.2f, k_i: %.2f\n\n", k_p, k_d, k_i);
  printf("ReferencePosition\n");
  printf("x_ref: %.f \t y_ref: %.f\n\n", x_ref, y_ref);
  printf("Position\n");
  printf("x: %.f \t y: %.f\n\n", x, y);
  printf("Reference Velocity\n");
  printf("vx_ref: %.f \t vy_ref: %.f\n\n", vx_ref, vy_ref);
  printf("Velocity\n");
  printf("vx: %.f \t vy: %.f\n\n", vx, vy);
  printf("Plate Angles\n");
  printf("pitch: %.f deg, roll: %.f deg \n\n", plate_angles[1], plate_angles[0]);
  printf("Servo Angles\n");
  printf("A: %.f deg, B: %.f deg, C: %.f deg\n\n", servo_angles[0], servo_angles[1], servo_angles[2]);
  printf("Integrators\n");
  printf("x_integ: %.2f, y_integ: %.2f\n", x_integrator, y_integrator);

  fprintf(
      fp,
      "%lu %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
      end_time, x_ref, y_ref, vx_ref, vy_ref, x_raw, y_raw, x, y, vx_raw,
      vy_raw, vx, vy);
  fclose(fp);

  return 0;
};
