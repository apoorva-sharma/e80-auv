
/*
 * File:   SensorGPS.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 10 June 2016
 */

#ifndef __SENSOR_GPS_H__
#define __SENSOR_GPS_H__

#include <Arduino.h>
#include <TinyGPS.h>
#include "DataSource.h"

typedef struct {
  long lat;
  long lon;
  unsigned long age;
  float hdop;
  int num_sat;
} gps_state_t;

class SensorGPS : public DataSource {
public:
  SensorGPS(HardwareSerial * Uart_p_);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor, returns whether new data was received
  bool read(void);

  // Latest reported data is stored here
  gps_state_t state;
  void printState(void);

  // from DataSource
  void getCSVString(String * csvStr_p);

private:
  TinyGPS gps;
  HardwareSerial * Uart_p;

  // copies over latest data from gps object to the state struct
  void updateState(void);
};

#endif