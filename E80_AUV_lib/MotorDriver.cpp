
/*
 * File:   MotorDriver.cpp
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#include "MotorDriver.h"

MotorDriver::MotorDriver(int left1, int left2, int right1, int right2)
  : right(0), left(0), left1(left1), left2(left2), right1(right1), right2(right2)
{
  // nothing
}

// TODO update to actually drive pins
void MotorDriver::apply(void)
{
  Serial.print("L: "); Serial.print(left);
  Serial.print("R: "); Serial.println(right);
}