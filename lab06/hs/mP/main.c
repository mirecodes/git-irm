#include <stdio.h>
#include "util.h"

int main()
{
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

  // Time information
  time_t rawtime;
  struct tm *info;
  char buffer[80];
  time(&rawtime);
  info = localtime(&rawtime);
  printf("Formatted date & time : |%s|\n", buffer);

  // Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("|%s|\n", buffer);
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
  const char *port = "/dev/cu.usbserial-01E4BAD5";
  int fd = serialport_init(port, 115200);
  if (fd == -1)
  {
    printf("Could not open the port.\n");
    printf(" - Is the Arduino IDE terminal opened?\n");
    printf(" - Is the device connected to the VM?\n");
    return -1;
  }

  // Initialize robot and check
  // if messages are received
  initBallBalancingRobot(fd);

  // Make sure that serial port is relaxed
  usleep(20 * 1000);

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

  if (task_selection == 1)
  {
    /* Test inverse kinematics via
     terminal */

    // initalize variables:
    double plate_angles[] = {0, 0};
    double servo_angles[] = {0, 0, 0};

    /* ********************* */
    /* Insert your Code here (From Lab 05)*/
    /* ********************* */
    /* Test inverse kinematics via terminal */

    int request = 1;
    while (request)
    {
      // #1 Get the angles from user
      printf("Input your angles in degree [phy_x] [theta_y]: ");
      scanf("%lf %lf", &plate_angles[0], &plate_angles[1]);

      // #2 Compute the servo angles
      int res = inverseKinematics(plate_angles, servo_angles);

      // #3 Send the command to the servos
      if (res == -1)
      { // servo angles are not feasible
        printf("Servo angles are not feasible\n");
      }
      else
      { // servo angles are feasible
        servoCommand(fd, servo_angles);
      }

      // #4 Print the angles
      printf("Plate angles: phi_x=%.2f, theta_y=%.2f\n", plate_angles[0], plate_angles[1]);
      printf("Servo angles: a_A=%.2f, a_B=%.2f, a_C=%.2f\n", servo_angles[0], servo_angles[1], servo_angles[2]);

      // #5 Ask user whether the process repeats
      int input = 0;
      int valid = 0;

      while (valid == 0)
      {
        printf("Do you want to repeat? [1]yes, [2]no: ");
        scanf("%d", &input);

        if (input == 1)
        {
          request = 1;
          valid = 1;
        }

        else if (input == 2)
        {
          request = 0;
          valid = 1;
        }

        else
        {
          printf("Input the right values\n");
          valid = 0;
        }
      }
    }
  }

  //////////////////////////////
  /////////// Task 2 ///////////
  //////////////////////////////
  /*Test camera calibration*/
  else if (task_selection == 2)
  {
    // initalize variables:

    /* ********************* */
    /* Insert your Code here (From Lab 05) */
    /* ********************* */

    // initalize variables:
    int flag = 0, x = 0, y = 0;
    double xout = 0.0, yout = 0.0;

    /* ********************* */
    /* Insert your Code here */
    /* ********************* */
    int input = 0;
    int valid = 0;

    while (!valid)
    {
      printf("Press [1]Continue, [2]Stop: ");
      scanf("%d", &input);

      if (input == 1)
      {
        readFromPixy(fd, &flag, &x, &y);

        if (flag == 0)
        {
          printf("Pixy camera detects nothing\n");
        }
        else if (flag == 1)
        {
          cameraCalibration(x, y, &xout, &yout);
          printf("x: %.2f [mm], y: %.2f [mm]\n", xout, yout);
        }
      }
      else if (input == 2)
      {
        break;
      }
      else
      {
        printf("Press the valid input\n");
      }
    }
  }

  //////////////////////////////
  /////// Task 4/5/6 ///////////
  //////////////////////////////

  if ((task_selection == 4) || (task_selection == 5) || (task_selection == 6))
  {

    // TODO: Initialize default PID parameters
    double k_p = 0.03;
    double k_d = 0;
    double k_i = 0;

    // TODO: Intialize filter window size
    int n_pos = 5;
    int n_vel = 10;

    // TODO: Ask for user input to change PID parameters
    /* ********************* */
    /* Insert your Code here */
    /* ********************* */

    int valid = 0;
    int choice = 0;
    while (!valid)
    {
      printf("Do you wish to use the default PID parameters or set your own? (0 for default, 1 for own): ");
      scanf("%d", &choice);
      if (choice == 0)
      {
        valid = 1;
      }
      else if (choice == 1)
      {
        printf("Please insert your PID parameters: (k_p) (k_d) (k_i)");
        scanf("%lf %lf %lf", &k_p, &k_d, &k_i);
        valid = 1;
      }
      else
      {
        printf("Type the correct input\n");
      }
    }

    // Variables for Pixy2
    int flag = 0;          // flag that detects if the pixy cam can detect a ball
    int x_px = 0;          // raw x coordinate read by pixy cam
    int y_px = 0;          // raw y coordinate read by pixy cam
    double x_cal = 0;      // calibrated x coordinate in plate frame and mm
    double y_cal = 0;      // calibrated y coordinate in plate frame and mm
    double x_filt = 0;     // filtered x coordinate in plate frame and mm
    double y_filt = 0;     // filtered y coordinate in plate frame and mm
    double vel_x = 0;      // x velocity calculated from filtered position
    double vel_y = 0;      // y velocity calculated from filtered position
    double vel_x_filt = 0; // filtered vel_x coordinate in plate frame and mm
    double vel_y_filt = 0; // filtered vel_y coordinate in plate frame and mm

    // read pixy a couple of times to clear buffer
    for (int i = 0; i < 20; i++)
    {
      readFromPixy(fd, &flag, &x_px, &y_px);
    }

    // create buffer arrays for filtered variables
    // [0] is always the current element
    // make sure buf_size is bigger than filter windows
    int buf_size = 50;
    double x_raw[buf_size]; // calibrated, unfiltered
    double y_raw[buf_size];
    double vx_raw[buf_size]; // 1st order derivative
    double vy_raw[buf_size];
    double x[buf_size]; // filtered position
    double y[buf_size];
    double vx[buf_size]; // filtered velocity
    double vy[buf_size];

    // initialize buffer arrays to zero
    for (int i = 0; i < buf_size; i++)
    {
      x_raw[i] = 0;
      y_raw[i] = 0;
      vx_raw[i] = 0;
      vy_raw[i] = 0;
      x[i] = 0;
      y[i] = 0;
      vx[i] = 0;
      vy[i] = 0;
    }

    // initialize angles
    double servo_angles[] = {0, 0, 0};
    double plate_angles[] = {0, 0};

    // reference variables for control
    // are being set in reference functions
    double x_ref, y_ref, vx_ref, vy_ref;

    // pid variables
    double x_integ = 0;
    double y_integ = 0;
    double u_x = 0;
    double u_y = 0;

    double u_p_x = 0;
    double u_p_y = 0;
    double u_d_x = 0;
    double u_d_y = 0;

    // Logfile with datetime as filename
    char datetime[80];
    strftime(datetime, 80, "%Y-%m-%d_%H-%M-%S_pid_log.txt", info);
    FILE *fp = fopen(datetime, "w+");
    startLogging(fp, task_selection, k_p, k_d, k_i, n_pos, n_vel);

    // Timing variables
    //  TODO: Measure the current sampling time dt (in seconds, for the derivative): It is the time it takes to run the previous loop iteration.
    //  Hint: use getMicroseconds() and don't forget to convert to seconds.
    long start = 0;
    long end = 16;
    long t0 = getMicroseconds(); // get starting time of the loop
    double dt = 0.016;           // variable for timing
    double current_time = 0;

    while (1)
    {
      /* ********************* */
      /* Insert your Code here */
      /* ********************* */

      // TODO: Get current sampling time dt
      end = start;
      start = getMicroseconds();
      current_time = (float)start / 1000000;
      if (start != t0)
      {
        dt = (float)(start - end) / 1000000;
      }

      // TODO: Get the coordinates of the ball in the Pixy Camera frame (Use a function in util.c)
      readFromPixy(fd, &flag, &x_px, &y_px);

      // If the ball is detected, enter if-bracket
      if (flag)
      {
        // TODO: Use camera calibration form Lab05
        cameraCalibration(x_px, y_px, &x_cal, &y_cal);
        // TODO: Place measurements in buffer array
        // Hint: There is a function called pushBack
        //  in util.h that you can use here.
        pushBack(x_cal, x_raw, buf_size);
        pushBack(y_cal, y_raw, buf_size);

        // TODO: Apply filter to position coordinates
        x_filt = movingAverage(n_pos, x_raw);
        y_filt = movingAverage(n_pos, y_raw);
        pushBack(x_filt, x, buf_size);
        pushBack(y_filt, y, buf_size);

        // TODO: Compute velocity based on the filtered position signal
        vel_x = discreteDerivative(dt, x);
        vel_y = discreteDerivative(dt, y);

        // TODO: Place velocity in buffer array (use pushBack function)
        pushBack(vel_x, vx_raw, buf_size);
        pushBack(vel_y, vy_raw, buf_size);

        // TODO: Apply filter to velocity
        vel_x_filt = movingAverage(n_vel, vx_raw);
        vel_y_filt = movingAverage(n_vel, vy_raw);
        pushBack(vel_x_filt, vx, buf_size);
        pushBack(vel_y_filt, vy, buf_size);

        // TODO: Set reference depending on task
        switch (task_selection)
        {
        case 4: /*TODO: Postlab Q4 centering task */
          x_ref = 0;
          y_ref = 0;
          vx_ref = 0;
          vy_ref = 0;
          break;
        case 5: /*TODO: Postlab Q5 step response reference  --> use function in util.h */
          stepResponse(current_time, &x_ref, &y_ref, &vx_ref, &vy_ref);
          break;
        case 6: /*TODO: Postlab Q6 circular trajectory reference --> implement & use function in util.h */
          circularTrajectory(current_time, &x_ref, &y_ref, &vx_ref, &vy_ref);
          break;
        }

        // TODO: Update Integrator after an initial delay
        // Hint: Wait 0.5s before starting to update integrator

        // u_p
        double u_p_x_prev = u_p_x;
        double u_p_y_prev = u_p_y;
        u_p_x = x_ref - x_filt;
        u_p_y = y_ref - y_filt;
        // u_d
        double u_d_x = vx_ref - vel_x_filt;
        double u_d_y = vy_ref - vel_y_filt;
        // u_i
        if (current_time > 0.5)
        { // 0.5s after starting
          x_integ += dt * (u_p_x + u_p_x_prev) / 2;
          y_integ += dt * (u_p_y + u_p_y_prev) / 2;
        }

        // TODO: Compute PID (remember, PID output is the plate angles)
        u_x = k_p * u_p_x + k_i * x_integ + k_d * u_d_x;
        u_y = k_p * u_p_y + k_i * y_integ + k_d * u_d_y;

        // TODO: Define Plate angles from PID output (watch out for correct sign)
        plate_angles[0] = -u_y; // u_y * 180 / M_PI
        plate_angles[1] = u_x;  // u_x * 180 / M_PI

        // TODO: Compute servo angles and send command
        inverseKinematics(plate_angles, servo_angles);
        servoCommand(fd, servo_angles);

        // Open logging file and log everything to textfile
        fp = fopen(datetime, "a");
        logger(fp, end, current_time, dt, k_p, k_d, k_i, x_ref, y_ref, vx_ref,
               vy_ref, x_raw[0], y_raw[0], x[0], y[0], vx_raw[0], vy_raw[0],
               vx[0], vy[0], plate_angles, servo_angles, x_integ, y_integ);
      }
    }
  }
  return 0;
}
