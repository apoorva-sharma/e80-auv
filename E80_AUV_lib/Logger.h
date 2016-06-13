
/*
 * File:   Logger.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 9 June 2016
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__

#define MAX_NUM_DATASOURCES 20
#define LOG_FILENAME_BASE "log"
#define LOG_FILENAME_BUFFERLEN 20

#include <Arduino.h>
#include "DataSource.h"

class Logger {
public:
  Logger(void);

  // include all dataSources before running init
  void include(DataSource * source_p);

  // run after all dataSources have been registered
  void init(void);

  // records a row of data with time value as given
  void log(unsigned long time_val);

private:
  DataSource* sources[MAX_NUM_DATASOURCES];
  unsigned int num_datasources;
  char logfilename[LOG_FILENAME_BUFFERLEN];

  void padding(int number, byte width, String & str);
};

#endif
