
/*
 * File:   Logger.cpp
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 10 June 2016
 */

#include "Params.h"
#include "Logger.h"
#include <SD.h>

Logger::Logger()
  : num_datasources(0)
{
}

void Logger::include(DataSource * source_p)
{
  sources[num_datasources] = source_p;
  ++num_datasources;
}

void Logger::padding(int number, byte width, String & str) {
  int currentMax = 10;
  for (byte i=1; i<width; i++){
   if (number < currentMax) {
     str.concat("0");
   }
   currentMax *= 10;
  }
  str.concat(number);
}

void Logger::init(void)
{
  // Determine logfilename by finding the first X such that basenameX is not
  // already on the SD card.
  unsigned int number = 0;
  String numstr = "";
  padding(number, 3, numstr);
  String finalname = LOG_FILENAME_BASE + numstr + ".txt";
  finalname.toCharArray(logfilename, LOG_FILENAME_BUFFERLEN);

  while(SD.exists(logfilename)) {
    number++;
    numstr = "";
    padding(number, 3, numstr);
    finalname = LOG_FILENAME_BASE + numstr + ".txt";
    finalname.toCharArray(logfilename, LOG_FILENAME_BUFFERLEN);
  }
  
  Serial.print("Logger: Using log file name "); Serial.println(logfilename);

  // Write column headings to file
  String headingStr = "time";
  for (size_t i = 0; i < num_datasources; ++i) {
    headingStr += ",";
    headingStr += sources[i]->csvHeadings;
  }

  File dataFile = SD.open(logfilename, O_WRITE | O_CREAT | O_APPEND);
  if (dataFile) {
    dataFile.println(headingStr);
    dataFile.close();
  } else {
    Serial.print("Logger: Error opening "); Serial.println(logfilename);
  }
}

void Logger::log(unsigned long time_val)
{
  String dataString = String(time_val);
  for (size_t i = 0; i < num_datasources; ++i) {
    dataString += ",";
    sources[i]->getCSVString(&dataString);
  }

  File dataFile = SD.open(logfilename, O_WRITE | O_CREAT | O_APPEND);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } else {
    Serial.print("Logger: Error opening "); Serial.println(logfilename);
  }
}
