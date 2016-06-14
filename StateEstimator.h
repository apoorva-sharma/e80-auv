
/*
 * File:   StateEstimator.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#ifndef __STATE_ESTIMATOR_H__
#define __STATE_ESTIMATOR_H__

#include <Arduino.h>

#include <SensorGPS.h>
#include <SensorIMU.h>
#include "MotorDriver.h"
#include "DataSource.h"


typedef struct {
  float x; // x position in global frame
  float y; // y position in global frame
  float heading; // heading in global frame
  float v; // linear velocity in robot frame
  float w; // rotational velocity in robot frame
} state_t;

/* 
 * StateEstimator class keeps track of the robot's state, incorporating 
 * measurements of the system outputs from the various sensors like IMU or 
 * GPS as well as the control inputs to the system.
 */
class StateEstimator : public DataSource
{
public:
  StateEstimator(void);

  // init
  void init(double loop_period, long orig_lat, long orig_lon);

  // Data incorporation
  void incorporateIMU(imu_state_t * imu_state_p);
  void incorporateGPS(gps_state_t * gps_state_p);
  void incorporateControl(MotorDriver* motorDriver_p);

  // State Access
  state_t state;
  void printState(void);

  void latlonToXY(long lat, long lon, float* x_p, float* y_p);

  // from DataSource
  void getCSVString(String * csvStr_p); // implement DataSource's virtual func

private:
  double loop_period;
  long orig_lat;
  long orig_lon;
  float cosOrigLat;
};

#endif