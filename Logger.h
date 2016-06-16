
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
#define HEADINGS_FILENAME_BASE "inf"
#define LOG_FILENAME_BUFFERLEN 20

#define MAX_BYTES_PER_ROW 500

#include <Arduino.h>
#include <SdFat.h>
#include "DataSource.h"

class Logger {
public:
  Logger(SdFat & sd_, SdFile & file_);

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
  char headingfilename[LOG_FILENAME_BUFFERLEN];

  SdFat & sd;
  SdFile & file;

  unsigned char rowbuffer[MAX_BYTES_PER_ROW];

  void padding(int number, byte width, String & str);
};

#endif
