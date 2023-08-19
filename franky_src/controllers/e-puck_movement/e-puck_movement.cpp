// File:          e-puck_movement.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/Device.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int time_step;

// Motor device
static WbDeviceTag left_motor, right_motor;
// E-puck angular speed in rad/s
#define MAX_SPEED 6.28;

//function to get simulator time step
static int get_time_step(){
  static int time_step = -1;
  if (time_step == -1)
  {
    time_step = wb_robot_get_basic_time_step();
  }
  return time_step;
}

//function to set motor velocity to move forward
static void move_forward(){
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

//function to init robot stuff
static void init_robot(){
  //get simulator time step
  time_step = get_time_step();
}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
