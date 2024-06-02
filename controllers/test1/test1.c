// #include <webots/robot.h>
// #include <webots/motor.h>
// #include <webots/distance_sensor.h>
// #include <webots/position_sensor.h>
// #include <webots/camera.h>
// #include <stdio.h>
// #include <time.h>
// #include <string.h>

// #define TIME_STEP 64
// #define MAX_SPEED 2.0

// // Define the distance sensors' indices
// #define NUM_SENSORS 8
// #define RIGHT_SENSOR 0
// #define RIGHT_FRONT_SENSOR 1
// #define FRONT_RIGHT_SENSOR 2
// #define FRONT_LEFT_SENSOR 5
// #define LEFT_FRONT_SENSOR 6
// #define LEFT_SENSOR 7

// r18g44b89
// #define WHEEL_RADIUS 0.02
// #define AXLE_LENGTH 0.052

// WbDeviceTag  left_position_sensor, right_position_sensor, left_motor, right_motor, sensors[NUM_SENSORS], accelerometer, camera;


// int camera_width;
// int camera_height;

// static double old_delta_orientation = 0.0;

// void format_date_time(char *output){
//     time_t rawtime;
//     struct tm * timeinfo;
    
//     time(&rawtime);
//     timeinfo = localtime(&rawtime);
    
//     sprintf(output, "%d-%d-%d %d:%d:%d", timeinfo->tm_mday,
//             timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,
//             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
// }

// void format_date_with_sec(char *output){
//     time_t rawtime;
//     struct tm * timeinfo;
    
//     time(&rawtime);
//     timeinfo = localtime(&rawtime);
    
//     sprintf(output, "%d_%d_%d_%d_%d_%d", 
//       timeinfo->tm_mday,
//       timeinfo->tm_mon + 1, timeinfo->tm_year + 1900,
//       timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
// }

// void get_center_of_camera(int *buffer, int camera_width, int camera_height) {
//   buffer[0] = camera_width/2;
//   printf("Buff %d \n", buffer[0]);
//   buffer[1] = camera_height/2;
// }

// int get_for_color_in_point(const unsigned char *image, int width, int height, char *sink) {
//   int r = wb_camera_image_get_red(image, camera_width, width, height);
//   int g = wb_camera_image_get_green(image, camera_width, width, height);
//   int b = wb_camera_image_get_blue(image, camera_width, width, height);
  
//   char timestamp[50] = {0};
//   format_date_time(timestamp);
  
//   // fprintf(stdin, "[%s], r=%d, g=%d, b=%d\n", timestamp, r, g, b);
//   if (sink != NULL) { // r g b
//     char lol[32] = {0};
//     sprintf(lol, "%d,%d,%d,", r, g, b);
//     strcat(sink, lol);  
//   }

//   return 0;
// }

// double angle_diff(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
//   double da = 0.0;
//   double l = wb_position_sensor_get_value(left_position_sensor);
//   double r = wb_position_sensor_get_value(right_position_sensor);
//   double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
//   double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
//   da = (dr - dl) / AXLE_LENGTH;  // delta orientation
//   // printf("estimated distance covered by left wheel: %g m.\n", dl);
//   // printf("estimated distance covered by right wheel: %g m.\n", dr);
//   // printf("estimated orientation: %g rad.\n", da);
//   double delta = da - old_delta_orientation;
//   // printf("estimated change of orientation: %g rad.\n", delta);
//   old_delta_orientation = da;
//   return delta;
// }
//
// int main(int argc, char **argv) {
//   char curr_date[50] = {0};
//   format_date_with_sec(curr_date); 
//   char file_path[100] = "./DIST_LOGS_";
//   strcat(file_path, curr_date);
//   strcat(file_path, ".csv");
//   printf("%s \n", file_path);

//   FILE *header = fopen(file_path, "w+");
//   fprintf(header, "time,ps0,ps1,ps2,ps3,ps4,ps5,ps6,ps7,rotation,r,g,b,\n");
//   fclose(header);

//   FILE *logs = fopen(file_path, "a");

//   int time_step;
//   int camera_time_step;
  
//   if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
//     printf("e-puck2 robot\n");
//     time_step = 64;
//     camera_time_step = 64;
//   } else {  // original e-puck
//     printf("e-puck robot\n");
//     time_step = 64;
//     camera_time_step = 1024;
//   }

//   int center[2] = {0};
  
//   /* initialize Webots */
//   wb_robot_init();

//   // Initialize motors
//   left_motor = wb_robot_get_device("left wheel motor");
//   right_motor = wb_robot_get_device("right wheel motor");

//   /* get a handler to the position sensors and enable them. */
//   left_position_sensor = wb_robot_get_device("left wheel sensor");
//   right_position_sensor = wb_robot_get_device("right wheel sensor");
//   wb_position_sensor_enable(left_position_sensor, time_step);
//   wb_position_sensor_enable(right_position_sensor, time_step);

//   camera = wb_robot_get_device("camera");
//   wb_camera_enable(camera, camera_time_step);
  
//   camera_width = wb_camera_get_width(camera);
//   camera_height = wb_camera_get_height(camera);
//   get_center_of_camera(center, camera_width, camera_height);

//   // accelerometer = wb_robot_get_device("accelerometer");
//   // wb_accelerometer_enable(accelerometer, time_step);

//   wb_motor_set_position(left_motor, INFINITY);
//   wb_motor_set_position(right_motor, INFINITY);
//   wb_motor_set_velocity(left_motor, 0.0);
//   wb_motor_set_velocity(right_motor, 0.0);

//   // Initialize distance sensors
//   char sensor_names[NUM_SENSORS][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
//   for (int i = 0; i < NUM_SENSORS; i++) {
//     printf("init >> %s \n", sensor_names[i]);
//     sensors[i] = wb_robot_get_device(sensor_names[i]);
//     wb_distance_sensor_enable(sensors[i], TIME_STEP);
//   }

//   while (wb_robot_step(TIME_STEP) != -1) {
//     char log_entry[512] = {0};
//     double angle = angle_diff(left_position_sensor, right_position_sensor);

//     const unsigned char *image = wb_camera_get_image(camera);

//     // Read sensor values
//     double sensor_values[NUM_SENSORS];
//     for (int i = 0; i < NUM_SENSORS; i++) {
//       sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
//     }

//     // Wall-following logic
//     double left_speed = MAX_SPEED * 0.5;
//     double right_speed = MAX_SPEED * 0.5;

//     if ((sensor_values[RIGHT_FRONT_SENSOR] >= 100) &&
//         (sensor_values[RIGHT_SENSOR] >= 100) &&
//         (sensor_values[LEFT_SENSOR] >= 100) && 
//         (sensor_values[LEFT_FRONT_SENSOR] >= 100)) {
               
//                     printf("HELLO \n");

//                     left_speed = -MAX_SPEED;
//                     right_speed = -MAX_SPEED;   
//                     wb_motor_set_velocity(left_motor, left_speed);
//                     wb_motor_set_velocity(right_motor, right_speed);
                    
//                     wb_robot_step(64);
                    
                    
//                     left_speed = -0.5*MAX_SPEED;
//                     right_speed = 0.5*MAX_SPEED;   
//                     wb_motor_set_velocity(left_motor, left_speed);
//                     wb_motor_set_velocity(right_motor, right_speed);
//                     continue;
//       } else if (sensor_values[LEFT_SENSOR] > 80 || sensor_values[LEFT_FRONT_SENSOR] > 80) {
//           // Turn RIGHT if there's a wall on the LEFT
//           left_speed += 0.5 * MAX_SPEED;
//           right_speed -= 0.5 * MAX_SPEED;
//       } else if (sensor_values[RIGHT_SENSOR] > 80 || sensor_values[RIGHT_FRONT_SENSOR] > 80) {
//           // Turn LEFT if there's a wall on the RIGHT
//           left_speed -= 0.5 * MAX_SPEED;
//           right_speed += 0.5 * MAX_SPEED;
//       } else if (sensor_values[FRONT_RIGHT_SENSOR] > 80 || sensor_values[FRONT_LEFT_SENSOR] > 80) {
//           // Turn around if there's a wall in front
//           left_speed = MAX_SPEED;
//           right_speed = -MAX_SPEED;
//       }

//     printf("%g -- %g -*- %g -- %g \n", sensor_values[RIGHT_FRONT_SENSOR], sensor_values[RIGHT_SENSOR], sensor_values[LEFT_SENSOR], sensor_values[LEFT_FRONT_SENSOR]);

//     // "time,ps0,ps1,ps2,ps3,ps4,ps5,ps6,ps7,rotation,r,g,b,left_speed,right_speed\n"

//     char timestamp[50] = {0};
//     format_date_time(timestamp);
//     strcat(log_entry, timestamp);
//     FILE *logs = fopen(file_path, "a");
//     fprintf(logs, "%s,", timestamp);

//     for (int i = 0; i <= 7; i++) {
//       char ttemp[32] = {0};
//       sprintf(ttemp, "%f,", sensor_values[i]);
//       strcat(log_entry, ttemp);
//      }

//     char tempp[32] = {0};
//     sprintf(tempp, "%f,", angle);
//     strcat(log_entry, tempp);

//     char temp[64] = {0};
//     get_for_color_in_point(image, center[0], center[1], temp);
//     strcat(log_entry, temp);
//     strcat(log_entry, "\n");

//     printf(log_entry);

//     // Set motor speeds
//     wb_motor_set_velocity(left_motor, left_speed);
//     wb_motor_set_velocity(right_motor, right_speed);

//   }

//   fclose(logs);
//   wb_robot_cleanup();
//   return 0;
// }

// ######################################################################################################33

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <string.h>
#include <stdio.h>

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

// Initialize motors
WbDeviceTag left_motor, right_motor, camera, sensors[NUM_SENSORS];

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

static int camera_width;
static int camera_height;

int check_for_color_in_point(const unsigned char *image, int width, int height) {
  int r = wb_camera_image_get_red(image, camera_width, width, height);
  int g = wb_camera_image_get_green(image, camera_width, width, height);
  int b = wb_camera_image_get_blue(image, camera_width, width, height);
  printf("r%dg%db%d at %dx%d \n", r, g, b, width, height);

// (r >= 63 && r <= 69) && (r >= 200 && r <= 210) &&
// (g >= 58 && g <= 64) && (g >= 178 && g <= 188) &&
// (b >= 20 && b <= 30) && (b >= 35 && b <= 42)

// g >= 58 && g <= 64
// g >= 178 && g <= 188

// b >= 20 && b <= 30
// b >= 35 && b <= 42

  if (
  // ((r >= 63 && r <= 69) || (r >= 200 && r <= 210)) &&
  // ((g >= 58 && g <= 64) || (g >= 178 && g <= 188)) &&
  // ((b >= 20 && b <= 30) || (b >= 35 && b <= 42))

  // ((r >= 210 && r <= 255)) &&
  // ((g >= 230 && g <= 245)) &&
  // ((b >= 0 && b <= 58))

  (((r >= 17 && r <= 19)) &&
  ((g >= 42 && g <= 45)) &&
  ((b >= 85 && b <= 91))) || (
  ((r >= 25 && r <= 36)) &&
  ((g >= 129 && g <= 132)) &&
  ((b >= 198 && b <= 201)))

  // orange and blue
  // ((r >= 63 && r <= 69) || (r >= 200 && r <= 210)) &&
  // ((g >= 58 && g <= 64) || (g >= 178 && g <= 188)) &&
  // ((b >= 20 && b <= 30) || (b >= 35 && b <= 42))

  ) {
    printf("FOUND MATCHING COLOR!!!: red=%d, green=%d, blue=%d \n", r, g, b);
    return 1;
  }

  return 0;
}

//lol

int main(int argc, char **argv) {
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

  // Initialize distance sensors
  char sensor_names[NUM_SENSORS][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    const unsigned char *image = wb_camera_get_image(camera);

    // int i, j;
    // for (i = 0; i < 5; i++) {
    //   for (j = 0; j < 2; j++) {
    //     printf("* %d\n", points_of_interests[i][j]);
    //   }
    // }

    for (int i = 0; i <= 4; i++) {
      // printf("%d x %d \n",  points_of_interests[i][0], points_of_interests[i][1]);
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
                   
                    continue;
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
  }

  wb_robot_cleanup();
  return 0;
}

