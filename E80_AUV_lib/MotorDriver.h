
/*
 * File:   MotorDriver.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include <Arduino.h>

/* 
 * MotorDriver handles the raw signals that are needed to drive the robot's
 * motors.
 */
class MotorDriver
{
public:
  MotorDriver(int left1, int left2, int right1, int right2);

  // applies the stored values to the pins
  void apply(void);

  // motor values (range from -127 to +127) for full reverse or full forward
  int right;
  int left;
private:
  // pins for the motors
  int left1;
  int left2;
  int right1;
  int right2;
};

#endif