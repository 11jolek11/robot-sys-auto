#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/position_sensor.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define TIME_STEP 64
#define MAX_SPEED 2.0

// Define the distance sensors' indices
#define NUM_SENSORS 8
#define LEFT_SENSOR 0
#define LEFT_FRONT_SENSOR 1
#define FRONT_LEFT_SENSOR 2
#define FRONT_RIGHT_SENSOR 5
#define RIGHT_FRONT_SENSOR 6
#define RIGHT_SENSOR 7

#define ROBOT_DIAMETER 0.0074
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052

// Roles
#define MASTER 1
#define SLAVE 0

// status
#define CONNECTED 1
#define NOT_CONNECTED 0

// Initialize motors
WbDeviceTag left_motor, right_motor, camera, sensors[NUM_SENSORS], recv, emit, left_position_sensor, right_position_sensor;
static double old_delta_orientation = 0.0;

void halt() {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}
// 52 x 39
int points_of_interests[5][2] = {
  {13, 13},
  {13, 26},
  {39, 13},
  {39, 26},

  {26, 19}
};

static int instance_id;

static double field_effective_range = 0.03;

static double local_da = 0.0;
static double diff_da = 3.0; // value == 3.0 --> bad value

static int power_key;

static int camera_width;
static int camera_height;

int check_for_color_in_point(const unsigned char *image, int width, int height) {
  int r = wb_camera_image_get_red(image, camera_width, width, height);
  int g = wb_camera_image_get_green(image, camera_width, width, height);
  int b = wb_camera_image_get_blue(image, camera_width, width, height);
  // printf("r%dg%db%d at %dx%d \n", r, g, b, width, height);

  if (
  (((r >= 17 && r <= 19)) &&
  ((g >= 42 && g <= 45)) &&
  ((b >= 85 && b <= 91))) || (
  ((r >= 25 && r <= 36)) &&
  ((g >= 129 && g <= 132)) &&
  ((b >= 198 && b <= 201)))
  ) {
    printf("FOUND MATCHING COLOR!!!: red=%d, green=%d, blue=%d \n", r, g, b);
    return 1;
  }

  return 0;
}

double angle_diff(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double da = 0.0;
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  da = (dr - dl) / AXLE_LENGTH;
  local_da = da / (2*M_PI);  // delta orientation
  // printf("estimated distance covered by left wheel: %g m.\n", dl);
  // printf("estimated distance covered by right wheel: %g m.\n", dr);
  // printf("estimated orientation: %g rad.\n", da);
  double delta = da - old_delta_orientation;
  // printf("estimated change of orientation: %g rad.\n", delta);
  old_delta_orientation = da;
  return delta;
}

int master(int transmit_angle) {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  MASTER >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    const unsigned char *image = wb_camera_get_image(camera);

    for (int i = 0; i <= 4; i++) {
      int detected = check_for_color_in_point(image, points_of_interests[i][0], points_of_interests[i][1]);
      if (detected) {
        halt();
        wb_robot_cleanup();
        return 0;
      }
    }

    // Read sensor values
    double sensor_values[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    // Wall-following logic
    double left_speed = MAX_SPEED * 0.5;
    double right_speed = MAX_SPEED * 0.5;

    if ((sensor_values[RIGHT_FRONT_SENSOR] >= 100) &&
        (sensor_values[RIGHT_SENSOR] >= 100) &&
        (sensor_values[LEFT_SENSOR] >= 100) && 
        (sensor_values[LEFT_FRONT_SENSOR] >= 100)) {
               
                    // printf("HELLO \n");

                    left_speed = -MAX_SPEED;
                    right_speed = -MAX_SPEED;   
                    wb_motor_set_velocity(left_motor, left_speed);
                    wb_motor_set_velocity(right_motor, right_speed);
                    
                    wb_robot_step(256);
                    
                    
                    left_speed = 0.5*MAX_SPEED;
                    right_speed = -0.5*MAX_SPEED;   
                    wb_motor_set_velocity(left_motor, left_speed);
                    wb_motor_set_velocity(right_motor, right_speed);
                    wb_robot_step(256);
                    halt();
                   
                    return 0;
      } else if (sensor_values[RIGHT_SENSOR] > 80 || sensor_values[RIGHT_FRONT_SENSOR] > 80) {
      // Turn left if there's a wall on the right
      left_speed += 0.5 * MAX_SPEED;
      right_speed -= 0.5 * MAX_SPEED;
    } else if (sensor_values[LEFT_SENSOR] > 80 || sensor_values[LEFT_FRONT_SENSOR] > 80) {
      // Turn right if there's a wall on the left
      left_speed -= 0.5 * MAX_SPEED;
      right_speed += 0.5 * MAX_SPEED;
    } else if (sensor_values[FRONT_LEFT_SENSOR] > 80 || sensor_values[FRONT_RIGHT_SENSOR] > 80) {
      // Turn around if there's a wall in front
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    }

    // Set motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    if (transmit_angle) {
      char temp[64] = {0};
      sprintf(temp,"A:%f\0", angle_diff(left_position_sensor, right_position_sensor));
      wb_emitter_send(emit, temp, strlen(temp)+1);
    }

    return 0;

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  MASTER <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

}

int slave() {
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  SLAVE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  halt();
  printf("I'm a slave\n");
  return 0;

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  SLAVE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
}

int main(int argc, char **argv) {
  int role = MASTER;
  int status = NOT_CONNECTED;
  instance_id = wb_robot_get_name()[6];
  power_key = rand() * instance_id;
  int time_step;
  int camera_time_step;
  
  /* initialize Webots */
  wb_robot_init();

  int recv_ch, emit_ch = 0;

  if (wb_robot_get_name()[6] == '1') {
    recv_ch = 5;
    emit_ch = 3;
  } else {
    recv_ch = 3;
    emit_ch = 5;
  }

  printf("Starting: ID: %d Recv CH %d Emit CH %d\n", instance_id, recv_ch, emit_ch);

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 64;
    camera_time_step = 1024;
  }

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);

  printf("Camera: %d x %d \n", camera_width, camera_height);

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  // Initialize distance sensors
  char sensor_names[NUM_SENSORS][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  recv = wb_robot_get_device("receiver");
  wb_receiver_enable(recv, TIME_STEP);
  wb_receiver_set_channel(recv, recv_ch);
  emit = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emit, emit_ch);
  wb_emitter_set_range(emit, ROBOT_DIAMETER + field_effective_range);

  while (wb_robot_step(TIME_STEP) != -1) {
    printf("HERE 2\n");

  // if (status) {
  //   char temp[64] = {0};
  //   sprintf(temp, "%f\0", local_da);
  //   wb_emitter_send(emit, temp, strlen(temp)+1);
  // }

  // if (wb_receiver_get_queue_length(recv) > 0 && status) {
  //   char* msg[64] = {0};
  //   strcpy(msg, (char *)wb_receiver_get_data(recv));

  //   double remote_da = 0.0;
  //   sscanf((char *)wb_receiver_get_data(recv), "%lf", &remote_da);
  //   wb_receiver_next_packet(recv);
  // }

  if (status == NOT_CONNECTED) {
    char temp[32];
    // sprintf(temp, "I%d\0", instance_id);
    sprintf(temp, "K%d\0", power_key);
    wb_emitter_send(emit, temp, strlen(temp)+1);
    printf("[%d]EMIT - pk: %d \n", instance_id, power_key);
  }

  if (wb_receiver_get_queue_length(recv) > 0) {
    char msg[64];
    strcpy(msg, (char *)wb_receiver_get_data(recv));
    wb_receiver_next_packet(recv);

    // if (strchr(msg, 'I') != NULL && status == NOT_CONNECTED) {
    //   wb_emitter_send(emit, "ACK\0", strlen("ACK")+1);
    //   status = CONNECTED;
    // }

    char * p;
    if (((p = strchr(msg, 'K')) != NULL) && status == NOT_CONNECTED) {
      ++p;
      int remote_power_key = atoi(p);

      if (power_key < remote_power_key) {
        role = SLAVE;
      }
      char temp[32];
      sprintf(temp, "I%d\0", role);
      wb_emitter_send(emit, temp, strlen(temp)+1);
    }

    if (((p = strchr(msg, 'I')) != NULL) && status == NOT_CONNECTED) {
      ++p;
      int remote_role = atoi(p);

      char com[32];
      if (role != remote_role) {
        // Is OK
        strcpy(com, "O1\0");
        wb_emitter_send(emit, com, strlen(com)+1);
      } else {
        // Not OK
        strcpy(com, "O1\0");
        wb_emitter_send(emit, com, strlen(com)+1);
      }
    }

    if (((p = strchr(msg, 'O')) != NULL) && status == NOT_CONNECTED) {
      ++p;
      int confirmation = atoi(p);

      if (confirmation) {
        status = CONNECTED;
      }
    }

    // double remote_da = 0.0;
    // sscanf((char *)wb_receiver_get_data(recv), "%lf", &remote_da);
    // wb_receiver_next_packet(recv);

  }

  if (role == MASTER) {
      master(0);
  } else {
    slave();
  }

  }

  wb_robot_cleanup();
  return 0;
}
// hello