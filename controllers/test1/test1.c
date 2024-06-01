#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// Define the distance sensors' indices
#define NUM_SENSORS 8
#define RIGHT_SENSOR 0
#define RIGHT_FRONT_SENSOR 1
#define FRONT_RIGHT_SENSOR 2
#define FRONT_LEFT_SENSOR 5
#define LEFT_FRONT_SENSOR 6
#define LEFT_SENSOR 7

int main(int argc, char **argv) {
  wb_robot_init();

  // Initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize distance sensors
  WbDeviceTag sensors[NUM_SENSORS];
  char sensor_names[NUM_SENSORS][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    // Read sensor values
    double sensor_values[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    // Wall-following logic
    double left_speed = MAX_SPEED * 0.5;
    double right_speed = MAX_SPEED * 0.5;

    if (sensor_values[LEFT_SENSOR] > 80 || sensor_values[LEFT_FRONT_SENSOR] > 80) {
      // Turn RIGHT if there's a wall on the LEFT
      left_speed += 0.5 * MAX_SPEED;
      right_speed -= 0.5 * MAX_SPEED;
    } else if (sensor_values[RIGHT_SENSOR] > 80 || sensor_values[RIGHT_FRONT_SENSOR] > 80) {
      // Turn LEFT if there's a wall on the RIGHT
      left_speed -= 0.5 * MAX_SPEED;
      right_speed += 0.5 * MAX_SPEED;
    } else if (sensor_values[FRONT_RIGHT_SENSOR] > 80 || sensor_values[FRONT_LEFT_SENSOR] > 80) {
      // Turn around if there's a wall in front
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else if (sensor_values[RIGHT_SENSOR] > 60 && sensor_values[LEFT_SENSOR] > 1000) {
      // Turn around if there's a wall in front
      printf("TICK\n");
    }

    printf("%g -- %g * %g -- %g \n", sensor_values[RIGHT_FRONT_SENSOR], sensor_values[RIGHT_SENSOR], sensor_values[LEFT_SENSOR], sensor_values[LEFT_FRONT_SENSOR]);
    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
