
/*
 * File:   MotorController.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include <Arduino.h>

#include "StateEstimator.h"
#include "VelocityController.h"
#include "MotorDriver.h"

/* 
 * MotorController computes the motor speeds necessary to achieve the velocity
 * setpoint provided.
 */
class MotorController
{
public:
  void control(StateEstimator * stateEstimator_p, velocity_setpoint_t * desiredVelocity_p, MotorDriver * driver_p);

private:
};

#endif