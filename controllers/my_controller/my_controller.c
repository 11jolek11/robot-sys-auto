#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <webots/emitter.h>
#include <webots/receiver.h>


#define VERBOSE_MOVEMENT (0x1 << 0)
#define VERBOSE_CAMERA_COLOR (0x1 << 1)

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)
#define DEFAULT_SPEED 5.0

#define TIME_STEP_PUCK 256
#define TIME_STEP_PUCK2 64

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

#define DISTANCE_TRIGGER 100

#define EMITTER_CHANNEL 3
#define RECV_CHANNEL 3

// #define TARGET_COLOR_R 255
// #define TARGET_COLOR_G 226
// #define TARGET_COLOR_B 7

#define TARGET_COLOR_R 100
#define TARGET_COLOR_G 100
#define TARGET_COLOR_B 100

WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor, emitter, receiver;

double speed = DEFAULT_SPEED;
double sensors_value[8];

int verbose = 0x0 + VERBOSE_CAMERA_COLOR;
// verbose = verbose + VERBOSE_MOVEMENT;

static int g_shape_guard = 0;

static int camera_width;
static int camera_height;

double signal_strength = 0.0;
double *vector_p;

// 52 x 39
int points_of_interests[5][2] = {
  {13, 13},
  {13, 26},
  {39, 13},
  {39, 26},

  {26, 19}
};

void halt() {
  if (verbose & VERBOSE_MOVEMENT) printf("#  HALT \n");
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

int sensor_blocked(double sensor, double distance) {
    if (sensor >= distance) {
        return 1;
    }
    return 0;
}

void turn_left() {
  if (verbose & VERBOSE_MOVEMENT) printf("#  LEFT \n");
  wb_motor_set_velocity(left_motor, -4.0);
  wb_motor_set_velocity(right_motor, 4.0);
  g_shape_guard--;
}

void turn_right() {
  if (verbose & VERBOSE_MOVEMENT) printf("#  RIGHT \n");
  wb_motor_set_velocity(left_motor, 4.0);
  wb_motor_set_velocity(right_motor, -4.0);
  g_shape_guard++;
}

void go() {
  if (verbose & VERBOSE_MOVEMENT)  printf("#  GO \n");
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
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

  if (
    // r >= TARGET_COLOR_R && g <= TARGET_COLOR_G && b <= TARGET_COLOR_B
    (((r >= 0 && r <= 15)) &&
    ((g >= 147 && g <= 152)) &&
    ((b >= 250 && b <= 255))) || (
    ((r >= 25 && r <= 36)) &&
    ((g >= 129 && g <= 132)) &&
    ((b >= 198 && b <= 201)))
    
    ) {
    if (verbose & VERBOSE_CAMERA_COLOR) printf("FOUND MATCHING COLOR!!!: red=%d, green=%d, blue=%d \n", r, g, b);
    return 1;
  }

  return 0;
}

int check_for_color_in_points_of_interests(const unsigned char *image) {
  for (int i = 0; i <= 4; i++) {
      if (check_for_color_in_point(image, points_of_interests[i][0], points_of_interests[i][1])) {
        return 1;
      }
  }
  return 0;
}

enum STATES {
  walking = 0,
  following = 1,
  followed = 2,
};

int main(int argc, char *argv[]) {
  /* define variables */
  int i, j;
  int time_step;
  int camera_time_step;
  
  enum STATES status = walking;
  int complete = 0;
  
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

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  int center[2] = {0};
  get_center_of_camera(center, camera_width, camera_height);
  
  printf("Width: %d camera_height: %d Camera center: %dx%d\n", camera_width, camera_height, center[0], center[1]);

  // Nie ma recognition
  // printf("Recong: %d \n", wb_camera_has_recognition(camera));
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, 1);
  wb_emitter_set_range(emitter, 0.1);
  srand(time(0) + (wb_robot_get_name()[6] - '0'));
  //int sent = rand();
  char sent[5];
  sent[0] = 111;
  sent[1] = (rand()%10) + '0';
  sent[2] = (rand()%10) + '0';
  sent[3] = (rand()%10) + '0';
  sent[4] = (rand()%10) + '\0';
  int sent_val;
  int received_val;
  sent_val = sent[3]-'0';
  sent_val += (sent[2]-'0')*10;
  sent_val += (sent[1]-'0')*100;

  receiver = wb_robot_get_device("receiver");
  wb_receiver_set_channel(receiver, 1);
  wb_receiver_enable(receiver, 1);
  
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

  printf("TO SEND: %s\n", sent);
  // main loop
  while (wb_robot_step(time_step) != -1) {
    const unsigned char *image = wb_camera_get_image(camera);
    // read sensors values
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
    
    int blocked_front = sensor_blocked(sensors_value[7], DISTANCE_TRIGGER) || sensor_blocked(sensors_value[0], DISTANCE_TRIGGER);
    int blocked_left = sensor_blocked(sensors_value[6], DISTANCE_TRIGGER);
    int blocked_left_close = sensor_blocked(sensors_value[5], 0.15);

    if (complete) {
        turn_right();
        wb_robot_step(time_step);
        //wb_emitter_send(emitter, "WW", 3);
        printf("Arrived!\n");
        continue;  
    // } else if (check_for_color_in_point(image, center[0], center[1]) && blocked_front) {
    } else if (check_for_color_in_points_of_interests(image) && blocked_front) {
      complete = 1;
    }
    
    if (status == followed) {
      printf("followed\n");

      // wb_emitter_set_channel(emitter, EMITTER_CHANNEL+3);
      wb_emitter_set_range(emitter, 2.6);

      char signal[5];
      signal[0] = 65;
      signal[1] = (rand()%10) + '0';
      signal[2] = (rand()%10) + '0';
      signal[3] = (rand()%10) + '0';
      signal[4] = (rand()%10) + '\0';

      wb_emitter_send(emitter, signal, strlen(signal));
      wb_robot_step(time_step);
      printf("NEW EMIT \n");
          
      // wb_emitter_set_channel(emitter, EMITTER_CHANNEL);
      // wb_emitter_set_range(emitter, 0.1);

    } else if (status == following) {
      printf("following\n");
      // wb_receiver_set_channel(receiver, RECV_CHANNEL + 3);
      wb_emitter_set_range(emitter, 2.6);

      int len = wb_receiver_get_queue_length(receiver);
      printf("LEN %d ON RECV CHN: %d \n", len, wb_receiver_get_channel(receiver));
      if (len > 0) {
        signal_strength = wb_receiver_get_signal_strength(receiver);
        printf("Signal power: %g  -  ", signal_strength);

        vector_p = wb_receiver_get_emitter_direction(receiver);
        printf("Signal vector: %g %g %g \n", vector_p[0], vector_p[1], vector_p[2]);

        wb_receiver_next_packet(receiver);
      }

      // wb_receiver_set_channel(receiver, RECV_CHANNEL);
      wb_emitter_set_range(emitter, 0.1);

    } else {
      printf("status: %d\n", status);

      if(status == walking) {
        // Handshake here
        
        while (true) {
          wb_emitter_send(emitter, sent, strlen(sent));
          wb_robot_step(time_step);
          printf("Sent: %s - %d\n", sent, sent_val);
          
          const char *received = wb_receiver_get_data(receiver);
          if (received == NULL) {
            break;
          }
          printf("Received: %s -", received);
          
          if (received[0] == 111) {
            received_val = received[3]-'0';
            received_val += (received[2]-'0')*10;
            received_val += (received[1]-'0')*100;
            printf(" %d\n", received_val);
            
            wb_receiver_next_packet(receiver);
            if (sent_val > received_val) {
              status = followed;
              turn_right();
              wb_robot_step(1000);    
              break;
            } else if (sent_val < received_val) {
              status = following;
              halt();
              wb_robot_step(1000);
              break;
            } else {
              sent[3] = (rand()%10) + '0';
            }
          }
          break;
        }
      } else if (status == following) {
        printf("Niewiem\n");
        // Prawdopodobnie obróc
        continue;
      }
    }
    if (status != following) {
    if (blocked_front) {
        turn_right();
        wb_robot_step(time_step);
        continue;
    } else if (blocked_left) {
        if (blocked_left_close) {
          turn_right();
          wb_robot_step(time_step);
        }
        go();
        wb_robot_step(time_step);
        continue;
    } else if (!blocked_left) {
        turn_left();
        wb_robot_step(time_step);
        continue;
    } else {
      printf("BBBBBBB\n");
      go();
      wb_robot_step(time_step);
      continue;
    }

    } else if (signal_strength >= 100.0) {
        if (vector_p[0] >= 0 && vector_p[1] >= 0) {
          turn_left();
          wb_robot_step(0.5*time_step);
          go();
          wb_robot_step(0.5*time_step);
          continue;
        } else if (vector_p[0] >= 0 && vector_p[1] < 0) {
          turn_right();
          wb_robot_step(0.5*time_step);
          go();
          wb_robot_step(0.5*time_step);
          continue;
        } else if (vector_p[0] < 0 && vector_p[1] >= 0) {
          turn_left();
          wb_robot_step(time_step);
          go();
          wb_robot_step(0.5*time_step);
          continue;
        } else if (vector_p[0] < 0 && vector_p[1] < 0) {
          turn_right();
          wb_robot_step(time_step);
          go();
          continue;
        } else {
          go();
          wb_robot_step(0.5*time_step);
          wb_robot_step(time_step);
        }

    }
  }

  wb_robot_cleanup();

  return 0;
}