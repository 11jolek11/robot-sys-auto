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

WbDeviceTag left_motor, right_motor;
double speed = DEFAULT_SPEED;

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


void get_center_of_camera(int *buffer, int camera_width, int camera_height) {
  buffer[0] = camera_width/2;
  printf("Buff %d \n", buffer[0]);
  buffer[1] = camera_height/2;
}

int get_for_color_in_point(const unsigned char *image, int width, int height, FILE *sink) {
  int r = wb_camera_image_get_red(image, camera_width, width, height);
  int g = wb_camera_image_get_green(image, camera_width, width, height);
  int b = wb_camera_image_get_blue(image, camera_width, width, height);
  
  char timestamp[50] = {0};
  format_date_time(timestamp);
  
  fprintf(stdin, "[%s], r=%d, g=%d, b=%d\n", timestamp, r, g, b);
  if (sink != NULL) { // r g b
    fprintf(sink, "%s,%d,%d,%d,\n", timestamp, r, g, b);  
  }

  return 0;
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

  char curr_date[50] = {0};
  format_date_with_sec(curr_date); 
  char file_path[100] = "./LOGS_";
  strcat(file_path, curr_date);
  strcat(file_path, ".csv");
  printf("%s \n", file_path);
  FILE *logs = fopen(file_path, "w+");
  fprintf(logs, "time,r,g,b,\n");
  fclose(logs);

  /* necessary to initialize webots stuff */
  wb_robot_init();
  
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

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  
  int center[2] = {0};
  get_center_of_camera(center, camera_width, camera_height);
  
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
  
    const unsigned char *image = wb_camera_get_image(camera);
    FILE *logs = fopen(file_path, "a");
    get_for_color_in_point(image, center[0], center[1], logs);
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
