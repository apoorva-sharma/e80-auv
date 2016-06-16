
/*
 * File:   StateEstimator.cpp
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#include "StateEstimator.h"
#include "Params.h"
#include <math.h>

#define RADIUS_OF_EARTH_M 6371000

StateEstimator::StateEstimator(void) 
  : DataSource(String("x,y,heading,v,w"),String("float,float,float,float,float")) // from DataSource
{}

void StateEstimator::init(double period, float lat, float lon)
{
  loop_period = period;
  orig_lat = lat;
  orig_lon = lon;
 	state.x = 0;
  state.y = 0;
  state.v = 0;
  state.w = 0;
  state.heading = 0;
  cosOrigLat = cos(orig_lat*M_PI/180.0);
}

void StateEstimator::incorporateIMU(imu_state_t * imu_state_p)
{
  float heading_rad = M_PI*imu_state_p->orientation.heading/180.0;
  heading_rad = -heading_rad + M_PI_2; // adjust from 0=North, CW=(+) to 0=East, CCW=(+)
  state.heading = fmod(heading_rad + M_PI, 2*M_PI) - M_PI;
}

void StateEstimator::incorporateGPS(gps_state_t * gps_state_p)
{
  float x;
  float y;

  latlonToXY(gps_state_p->lat, gps_state_p->lon, &x, &y);

  state.x = x;
  state.y = y;
}

inline float modfloat(float x, float m) {
  float r = fmod(x,m);
  return (r<0) ? r + m : r;
}

void StateEstimator::incorporateControl(MotorDriver* motorDriver_p)
{
  float sum = (float) motorDriver_p->right + motorDriver_p->left;
  float diff = (float) motorDriver_p->right - motorDriver_p->left;

  state.v = sum / LIN_VEL_CONST;
  state.w = diff / ROT_VEL_CONST;

  // forward euler update
  float vx = state.v*cos(state.heading);
  float vy = state.v*sin(state.heading);

  state.x += vx*loop_period;
  state.y += vy*loop_period;
  state.heading += state.w;
  state.heading = modfloat(state.heading + M_PI, 2*M_PI) - M_PI;
}

void StateEstimator::printState(void)
{
  Serial.print("x:"); Serial.print(state.x);
  Serial.print(" y:"); Serial.print(state.y);
  Serial.print(" h:"); Serial.print(state.heading);
  Serial.print(" v:"); Serial.print(state.v);
  Serial.print(" w:"); Serial.println(state.w);
}

void StateEstimator::getCSVString(String * csvStr_p)
{
  *csvStr_p += String(state.x);
  *csvStr_p += ","; *csvStr_p += String(state.y);
  *csvStr_p += ","; *csvStr_p += String(state.heading);
  *csvStr_p += ","; *csvStr_p += String(state.v);
  *csvStr_p += ","; *csvStr_p += String(state.w);
}

size_t StateEstimator::writeDataBytes(unsigned char * buffer, size_t idx)
{
  float * data_slot = (float *) (buffer + idx);
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  data_slot[2] = state.heading;
  data_slot[3] = state.v;
  data_slot[4] = state.w;
  return idx + 5*sizeof(float);
}

void StateEstimator::latlonToXY(float lat, float lon, float* x, float* y)
{
  *x = (lon-orig_lon)*M_PI/180.0*RADIUS_OF_EARTH_M*cosOrigLat;
  *y = (lat-orig_lat)*M_PI/180.0*RADIUS_OF_EARTH_M;
}

