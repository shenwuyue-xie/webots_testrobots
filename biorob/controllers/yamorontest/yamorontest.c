#include <math.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/device.h>




#define CONTROL_STEP 32
#define N 8

static double t = 0.0; /* time elapsed since simulation start [s] */
static int id = -1;    /* this module's ID */

static WbDeviceTag motor, rear_connector, front_connector;


/* worm movement oscillations */
static const double F = 1.0; /* frequency */
static const double A = 0.9; /* amplitude */

static int sec = 1;

static int duration = 10;

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

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

static void disconnect_0_and_7(){
  static int done = 0;

  if (done)
    return;
  
  if (id == 0)
    wb_connector_unlock(front_connector);
  if (id == 7)
    wb_connector_unlock(rear_connector);
  
  done = 1;
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
  
  double start_time = wb_robot_get_time();
  while (wb_robot_step(CONTROL_STEP)!=-1){
    if( t > start_time + duration){
      break;
    }
    else
    {
      t += CONTROL_STEP / 1000.0;
      worm_8_modules();
    }
  }

  start_time = wb_robot_get_time();
  
  while (wb_robot_step(CONTROL_STEP)!=-1){
    if( t > start_time + sec){
      break;
    }
    else
    {
      t += CONTROL_STEP / 1000.0;
      loop_8_modules();
    }
  }

  passive_wait(1);
  
  start_time = wb_robot_get_time();
  
  while (wb_robot_step(CONTROL_STEP)!=-1){
    if( t > start_time + sec){
      break;
    }
    else
    {
      t += CONTROL_STEP / 1000.0;
      connect_0_and_7();
    }
  }
  
  start_time = wb_robot_get_time();
  
  while (wb_robot_step(CONTROL_STEP)!=-1){
    if( t > start_time + duration){
      break;
    }
    else
    {
      t += CONTROL_STEP / 1000.0;
      loop_8_modules();
    }
  }
  passive_wait(1);
  
  start_time = wb_robot_get_time();
  
  while (wb_robot_step(CONTROL_STEP)!=-1){
    if( t > start_time + sec){
      break;
    }
    else
    {
      t += CONTROL_STEP / 1000.0;
      disconnect_0_and_7();
    }
  }
  
  start_time = wb_robot_get_time();
  
  while (wb_robot_step(CONTROL_STEP)!=-1){
    if( t > start_time + duration){
      break;
    }
    else
    {
      t += CONTROL_STEP / 1000.0;
      worm_8_modules();
    }
  }
  
  

  wb_robot_cleanup();
  return 0;
}