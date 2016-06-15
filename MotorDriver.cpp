
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

}

void MotorDriver::apply(void)
{
  Serial.print("L: "); Serial.print(left);
  Serial.print("R: "); Serial.println(right);

  // determine direction of spin required:
  unsigned char l_abs = (left < 0) ? -2*left : 2*left;
  unsigned char r_abs = (right < 0) ? -2*right : 2*right;
  analogWrite(left1, (left >= 0)*l_abs); // forward pin
  analogWrite(left2, (left < 0)*l_abs); // backward pin
  analogWrite(right1, (right >= 0)*r_abs); // forward pin
  analogWrite(right2, (right < 0)*r_abs); // backward pin

  Serial.print("L: "); Serial.print((left >= 0)*l_abs); Serial.print(" "); Serial.println((left < 0)*l_abs);
  Serial.print("R: "); Serial.print((right >= 0)*r_abs); Serial.print(" "); Serial.println((right < 0)*r_abs);
}