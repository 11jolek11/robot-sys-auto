/*
 * File:          color_probe.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>
#include <time.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define DEFAULT_SPEED 3.0

#define VERBOSE_MOVEMENT (0x1 << 0)
#define VERBOSE_CAMERA_COLOR (0x1 << 1)

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)
#define DEFAULT_SPEED 3.0

#define TIME_STEP_PUCK 256
#define TIME_STEP_PUCK2 64

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

#define DISTANCE_TRIGGER 110
#define TIME_TO_TURN 256*4 + 32
#define ACTION_DELAY 256 + 64

#define TARGET_COLOR_R 70
#define TARGET_COLOR_G 70
#define TARGET_COLOR_B 30

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052

WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
double speed = DEFAULT_SPEED;

double sensors_value[8];

static int camera_width;
static int camera_height;

void format_date_time(char *output){
    time_t rawtime;
    struct tm * timeinfo;
    
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    
    sprintf(output, "%d-%d-%d %d:%d:%d", timeinfo->tm_mday,
            timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

void format_date_with_sec(char *output){
    time_t rawtime;
    struct tm * timeinfo;
    
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    
    sprintf(output, "%d_%d_%d_%d_%d_%d", 
      timeinfo->tm_mday,
      timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,
      timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

double compute_distance_left_wheel(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  return l * WHEEL_RADIUS;         // distance covered by left wheel in meter
}

double compute_distance_right_wheel(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double r = wb_position_sensor_get_value(right_position_sensor);
  return r * WHEEL_RADIUS;         // distance covered by right wheel in meter
}

void halt() {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

void go() {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
}

void reverse() {
  wb_motor_set_velocity(left_motor, (-1)*speed);
  wb_motor_set_velocity(right_motor, (-1)*speed);
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  // wb_robot_init();

  char curr_date[50] = {0};
  format_date_with_sec(curr_date); 
  char file_path[100] = "./DIST_LOGS_";
  strcat(file_path, curr_date);
  strcat(file_path, ".csv");
  printf("%s \n", file_path);
  FILE *logs = fopen(file_path, "w+");
  fprintf(logs, "time,0,1,2,3,4,5,6,7,left_distance,right_distance,\n");
  fclose(logs);

  /* necessary to initialize webots stuff */
  // wb_robot_init();
  
  int time_step;
  int camera_time_step;
  
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 64;
    camera_time_step = 1024;
  }

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (int i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }
  
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  halt();

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    go();

    for (int i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }

    char timestamp[50] = {0};
    format_date_time(timestamp);

    FILE *logs = fopen(file_path, "a");

    fprintf(logs, "%s,", timestamp);

    for (int i = 0; i <= 7; i++) {
      char temp[32] = {0};
      sprintf(temp, "%f,", sensors_value[i]);
      fprintf(logs, temp);
    }

    double real_distance[2] = {0.0};

    real_distance[0] = compute_distance_left_wheel(left_position_sensor, right_position_sensor);
    real_distance[1] = compute_distance_right_wheel(left_position_sensor, right_position_sensor);

    for (int i = 0; i <= 1; i++) {
      char temp[32] = {0};
      sprintf(temp, "%f,", real_distance[i]);
      fprintf(logs, temp);
    }

    fprintf(logs, "\n");

    fclose(logs);
    go();
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  fclose(logs);
  wb_robot_cleanup();

  return 0;
}
