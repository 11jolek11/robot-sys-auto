#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

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

#define DISTANCE_TRIGGER 1000
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

int verbose = 0x0;
// verbose = verbose + VERBOSE_MOVEMENT;

static int g_shape_guard = 0;

static int camera_width;
static int camera_height;

static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);
}

static double old_delta_orientation = 0.0;

double angle_diff(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double da = 0.0;
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  // printf("estimated distance covered by left wheel: %g m.\n", dl);
  // printf("estimated distance covered by right wheel: %g m.\n", dr);
  // printf("estimated orientation: %g rad.\n", da);
  double delta = da - old_delta_orientation;
  // printf("estimated change of orientation: %g rad.\n", delta);
  old_delta_orientation = da;
  return delta;
}

void halt() {
  if (verbose & VERBOSE_MOVEMENT) printf("#  HALT \n");
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

int is_blocked(double sensor) {
    return sensor >= DISTANCE_TRIGGER;
}

void turn_left() {
  halt();
  if (verbose & VERBOSE_MOVEMENT) printf("#  LEFT \n");
  wb_motor_set_velocity(left_motor, -2.0);
  wb_motor_set_velocity(right_motor, 2.0);
  wb_robot_step(TIME_TO_TURN);
  g_shape_guard--;
  halt();
}

void turn_right() {
  halt();
  if (verbose & VERBOSE_MOVEMENT) printf("#  RIGHT \n");
  wb_motor_set_velocity(left_motor, 2.0);
  wb_motor_set_velocity(right_motor, -2.0);
  wb_robot_step(TIME_TO_TURN);
  g_shape_guard++;
  halt();
}

void turn_left_by_angle(double rad_angle, WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  halt();
  if (verbose & VERBOSE_MOVEMENT) printf("#  LEFT \n");
  printf(">> \n");
  double total_rad = 0.0;
  while(total_rad <= rad_angle) {
    printf("> \n");
    wb_motor_set_velocity(left_motor, -2.0);
    wb_motor_set_velocity(right_motor, 2.0);
    wb_robot_step(1);
    total_rad = total_rad + angle_diff(left_position_sensor, right_position_sensor);
  };
  // angle_diff(left_position_sensor, right_position_sensor)
  printf(">> Total angle change: %g \n", total_rad);
  g_shape_guard--;
  halt();
}

void turn_right_by_angle(double rad_angle, WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  halt();
  if (verbose & VERBOSE_MOVEMENT) printf("#  LEFT \n");
  printf(">> \n");
  rad_angle = -rad_angle;
  double total_rad = 0.0;
  while(total_rad > rad_angle) {
    printf("T - %g\n", total_rad);
    wb_motor_set_velocity(left_motor, 2.0);
    wb_motor_set_velocity(right_motor, -2.0);
    wb_robot_step(1);
    total_rad = total_rad + angle_diff(left_position_sensor, right_position_sensor);
  };
  printf(">> %g \n", angle_diff(left_position_sensor, right_position_sensor));
  g_shape_guard--;
  halt();
}

void go() {
  if (verbose & VERBOSE_MOVEMENT)  printf("#  GO \n");
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
  wb_robot_step(ACTION_DELAY*5);
}

void reverse() {
  if (verbose & VERBOSE_MOVEMENT) printf("#  REVERSE \n");
  wb_motor_set_velocity(left_motor, (-1)*speed);
  wb_motor_set_velocity(right_motor, (-1)*speed);
}

void get_center_of_camera(int *buffer, int camera_width, int camera_height) {
  buffer[0] = camera_width/2;
  printf("Buff %d \n", buffer[0]);
  buffer[1] = camera_height/2;
}

int check_for_color_in_point(const unsigned char *image, int width, int height) {
  int r = wb_camera_image_get_red(image, camera_width, width, height);
  int g = wb_camera_image_get_green(image, camera_width, width, height);
  int b = wb_camera_image_get_blue(image, camera_width, width, height);
  if (verbose & VERBOSE_CAMERA_COLOR) printf("red=%d, green=%d, blue=%d \n", r, g, b);

// (r >= 63 && r <= 69) && (r >= 200 && r <= 210) &&
// (g >= 58 && g <= 64) && (g >= 178 && g <= 188) &&
// (b >= 20 && b <= 30) && (b >= 35 && b <= 42)

// g >= 58 && g <= 64
// g >= 178 && g <= 188

// b >= 20 && b <= 30
// b >= 35 && b <= 42

  if (
  (r >= 63 && r <= 69) && (r >= 200 && r <= 210) &&
  (g >= 58 && g <= 64) && (g >= 178 && g <= 188) &&
  (b >= 20 && b <= 30) && (b >= 35 && b <= 42)
  ) {
    if (verbose & VERBOSE_CAMERA_COLOR) printf("FOUND MATCHING COLOR!!!: red=%d, green=%d, blue=%d \n", r, g, b);
    return 1;
  }

  return 0;
}

int main(int argc, char *argv[]) {
  /* define variables */
  int i, j;
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
    time_step = 32;
    camera_time_step = 1024;
  }

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  int center[2] = {0};
  get_center_of_camera(center, camera_width, camera_height);
  
  printf("Width: %d camera_height: %d Camera center: %dx%d\n", camera_width, camera_height, center[0], center[1]);

  // Nie ma modulu recognition dla kamery
  // printf("Recong: %d \n", wb_camera_has_recognition(camera));

  
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  halt();

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }
  printf("Starting...\n");
  /* compute odometry */
  angle_diff(left_position_sensor, right_position_sensor);
  printf("-----------\n");

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  // main loop
  while (wb_robot_step(time_step) != -1) {
    const unsigned char *image = wb_camera_get_image(camera);
    // read sensors values
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
    
    if (verbose & VERBOSE_MOVEMENT) printf("Distance: %f # %f\n", sensors_value[0], sensors_value[7]);
    
    int sensors_status[8] = {0};
    for (int i = 0; i <= 7; i++) {
      sensors_status[i] = is_blocked(sensors_value[i]);
    }

    check_for_color_in_point(image, center[0], center[1]);
    halt();
    wb_robot_step(TIME_TO_TURN);
    if (!sensors_status[5] && !(sensors_status[0] && sensors_status[7])) {
      printf("1\n");
      halt();
      // turn_left();
      turn_left_by_angle(1.5708, left_position_sensor, right_position_sensor);
      halt();
      go();
      wb_robot_step(ACTION_DELAY);
      /* compute odometry */
      angle_diff(left_position_sensor, right_position_sensor);
      continue;
    } else if (sensors_status[5] && (!sensors_status[0] && !sensors_status[7])) {
      printf("2\n");
      halt();
      go();
      wb_robot_step(ACTION_DELAY);
      halt();
      continue;
    } else if (!sensors_status[5] && (sensors_status[0] && sensors_status[7])) {
      printf("3\n");
      halt();
      // turn_left();
      turn_left_by_angle(1.5708, left_position_sensor, right_position_sensor);
      halt();
      go();
      wb_robot_step(ACTION_DELAY);

      /* compute odometry */
      angle_diff(left_position_sensor, right_position_sensor);
      continue;
    } else if (sensors_status[5] && (sensors_status[0] && sensors_status[7])) {
      printf("4\n");
      halt();
      // turn_right();
      turn_right_by_angle(1.5708, left_position_sensor, right_position_sensor);
      halt();
      go();
      wb_robot_step(ACTION_DELAY);
      /* compute odometry */
      angle_diff(left_position_sensor, right_position_sensor);
      continue;
    }
  }
  
  wb_robot_cleanup();

  return 0;
}
