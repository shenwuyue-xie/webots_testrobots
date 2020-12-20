#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 6

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.0

/* virtual time between two calls to the run() function */
#define CONTROL_STEP 32

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */

/* control types */
enum { AUTO, STOP };
static int control = AUTO;

/* locomotion types */
enum {
  WALK,
  SWIM,
};
static int locomotion = WALK;

/* motors position range */
static double min_motor_position[NUM_MOTORS];
static double max_motor_position[NUM_MOTORS];

double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  } else if (min == 0 && max == 0) {
    return value;
  }

  return value < min ? min : value > max ? max : value;
}


int main() {
  const double FREQUENCY = 1.4; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.0; /* radians */

  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /* distance sensors and gps devices */
  WbDeviceTag ds_left, ds_right, gps;

  /* Initialize Webots lib */
  wb_robot_init();

  /* for loops */
  int i;

  /* get the motors device tags */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6"};
  for (i = 0; i < NUM_MOTORS; i++) {
    motor[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    min_motor_position[i] = wb_motor_get_min_position(motor[i]);
    max_motor_position[i] = wb_motor_get_max_position(motor[i]);
  }

  /* get and enable left and right distance sensors */
  ds_left = wb_robot_get_device("ds_left");
  wb_distance_sensor_enable(ds_left, CONTROL_STEP);
  ds_right = wb_robot_get_device("ds_right");
  wb_distance_sensor_enable(ds_right, CONTROL_STEP);

  /* get and enable gps device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, CONTROL_STEP);

  /* enable keyboard */
  wb_keyboard_enable(CONTROL_STEP);

  /* control loop: sense-compute-act */
  while (wb_robot_step(CONTROL_STEP) != -1) {
    

    if (control == AUTO) {
      /* perform sensor measurment */
      double left_val = wb_distance_sensor_get_value(ds_left);
      double right_val = wb_distance_sensor_get_value(ds_right);

      /* change direction according to sensor reading */
      spine_offset = (right_val - left_val);
  
      /* increase phase according to elapsed time */
      phase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

      /* get current elevation from gps */
      double elevation = wb_gps_get_values(gps)[Y];

      if (locomotion == SWIM && elevation > WATER_LEVEL - 0.003) {
        locomotion = WALK;
        phase = target_position[5];
      } else if (locomotion == WALK && elevation < WATER_LEVEL - 0.015) {
        locomotion = SWIM;      }

      /* switch locomotion control according to current robot elevation and water level */
      if (locomotion == WALK) {
        /* above water level: walk (s-shape of robot body) */
  
        for (i = 0; i < 6; i++)
          target_position[i] = WALK_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;

      } 
      else { /* SWIM */
        /* below water level: swim (travelling wave of robot body) */
        for (i = 0; i < 6; i++)
          target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
      }
    }

    /* motors actuation */
    for (i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
  }

  wb_robot_cleanup();

  return 0; /* this statement is never reached */
}
