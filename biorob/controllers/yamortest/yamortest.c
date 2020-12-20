#include <math.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/device.h>
#include <webots/distance_sensor.h>

#define CONTROL_STEP 32
#define N 8

static double t = 0.0; /* time elapsed since simulation start [s] */
static int id = -1;    /* this module's ID */

static WbDeviceTag motor, rear_connector, front_connector, distance_sensor;
static double distance_sensor_value;

/* worm movement oscillations */
static const double F = 1.0; /* frequency */
static const double A = 0.9; /* amplitude */

static void loop_8_modules() {
  /*
   * Lookup positions for loop_8_modules locomotion
   * 4 motors are straight and 4 motors move to an angle of 90
   */
  const double LOOKUP[8] = {1, 1, 0, 0, 1, 1, 0, 0};

  wb_motor_set_position(motor, -1.5708 * LOOKUP[(id + (int)t) % 8]);
}

static void worm_8_modules() {
  double shift = id * (2.0 * M_PI / 8); /* phase shift for this module */
  double phase = 2.0 * M_PI * F * t;

  wb_motor_set_position(motor, A * sin(phase + shift));
}

static void connect_0_and_7() {
  static int done = 0;

  if (done)
    return;

  /* modules 0 and 7 should be aligned: try to lock */
  if (id == 0)
    wb_connector_lock(front_connector);
  if (id == 7)
    wb_connector_lock(rear_connector);

  done = 1;
}

static void disconct_0_and_7(){
  static int done = 0;

  if (done)
    return;
  
  if (id == 0)
    wb_connector_unlock(front_connector);
  if (id == 7)
    wb_connector_unlock(rear_connector);
  
  done = 1;
}

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}



int main(){
  wb_robot_init();

  /*
   * Find module id from robot name
   * The robot name is used to identify each module
   */
  const char *name = wb_robot_get_name();

  id = atoi(name + 7) - 1;

  /* find hardware devices */
  motor = wb_robot_get_device("motor");
  rear_connector = wb_robot_get_device("rear_connector");
  front_connector = wb_robot_get_device("front_connector");
  distance_sensor = wb_robot_get_device("d1");
  wb_distance_sensor_enable(distance_sensor, get_time_step());
  while (wb_robot_step(CONTROL_STEP) != -1) {

    worm_8_modules();
    
    bool obstacle = false;

    if (id == 0 ){
      distance_sensor_value= wb_distance_sensor_get_value("d1");
    
      bool obstacle = distance_sensor_value >= 80;
    };

    

    if (obstacle){
      connect_0_and_7();
      passive_wait(1);
      double start_time = wb_robot_get_time();
      sec = 10;

      while (start_time + sec>  wb_robot_get_time()) {
        loop_8_modules();
      }

    }
    worm_8_modules();
  };
  return EXIT_SUCCESS;
}