
/*
 * File:   DataSource.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 9 June 2016
 */

#ifndef __DATA_SOURCE_H__
#define __DATA_SOURCE_H__

#include <Arduino.h>

class DataSource {
public:
  // appends data values separated by commas, with no commas on either end to csvStr
  virtual void getCSVString(String * csvStr_p) = 0;

  // contains comma separated headings for each datavalue the dataSource reports
  const String csvHeadings;

protected:
  DataSource(String headingStr) : csvHeadings(headingStr) {}
};

#endif
