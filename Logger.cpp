
/*
 * File:   Logger.cpp
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 10 June 2016
 */

#include "Params.h"
#include "Logger.h"

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
  String finalname = LOG_FILENAME_BASE + numstr + ".bin";
  finalname.toCharArray(logfilename, LOG_FILENAME_BUFFERLEN);

  while(SD.exists(logfilename)) {
    number++;
    numstr = "";
    padding(number, 3, numstr);
    finalname = LOG_FILENAME_BASE + numstr + ".bin";
    finalname.toCharArray(logfilename, LOG_FILENAME_BUFFERLEN);
  }

  // determine corresponding headingfilename
  finalname = HEADINGS_FILENAME_BASE + numstr + ".txt";
  finalname.toCharArray(headingfilename, LOG_FILENAME_BUFFERLEN);
  
  Serial.print("Logger: Using log file name "); Serial.println(logfilename);

  // Write column headings and datatypes to file
  String headingStr = "time";
  String dataTypeStr = "ul";
  for (size_t i = 0; i < num_datasources; ++i) {
    headingStr += ",";
    headingStr += sources[i]->csvVarNames;
    dataTypeStr += ",";
    dataTypeStr += sources[i]->csvDataTypes;
  }
  headingStr += "\n";
  headingStr += dataTypeStr;

  dataFile = SD.open(headingfilename, O_WRITE | O_CREAT | O_APPEND);
  if (dataFile) {
    dataFile.println(headingStr);
    dataFile.close();
  } else {
    Serial.print("Logger: Error opening "); Serial.println(headingfilename);
    while (1) {}; // stop the show
  }

  // open the log file to be ready to write
  dataFile = SD.open(logfilename, O_WRITE | O_CREAT | O_APPEND);
  if (!dataFile) {
    Serial.print("Logger: Error opening "); Serial.println(logfilename);
    while (1) {}; // stop the show
  }
  // leave the file open
}

void Logger::log(unsigned long time_val)
{
  unsigned long t1 = micros();
  size_t idx = 0;
  unsigned long * time_slot = (unsigned long *) rowbuffer;
  *time_slot = time_val;
  idx += sizeof(time_val);

  for (size_t i = 0; i < num_datasources; ++i) {
    idx = sources[i]->writeDataBytes(rowbuffer, idx);
  }
  unsigned long t2 = micros();
  dataFile.write(rowbuffer, idx);
  dataFile.flush();
  unsigned long t3 = micros();
  Serial.print("generating buffer took (us): "); Serial.println(t2-t1);
  Serial.print("writing buffer to card took (us): "); Serial.println(t3-t2);
}
