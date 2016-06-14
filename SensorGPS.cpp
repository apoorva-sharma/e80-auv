
/*
 * File:   SensorGPS.cpp
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 10 June 2016
 */

#include "SensorGPS.h"
#include "Params.h"

SensorGPS::SensorGPS(HardwareSerial * Uart_p_)
  : DataSource("lat,lon,hdop,nsats"), Uart_p(Uart_p_)
{  
}

void SensorGPS::init(void)
{
  Uart_p->begin(9600);
}

bool SensorGPS::read(void)
{
  unsigned long start_time = millis();
 
  while (millis() - start_time < GPS_READ_INTERVAL) {
    if (Uart_p->available()) {
      char c = Uart_p->read();
      if (gps.encode(c)) {
        updateState();
        return 1;
      }
    }
  }
  return 0;
}

void SensorGPS::printState(void)
{
  Serial.print("GPS: Lat:"); Serial.print(state.lat);
  Serial.print(" Lon:"); Serial.print(state.lon);
  Serial.print(" HDOP:"); Serial.print(state.hdop);
  Serial.print(" N sats:"); Serial.print(state.num_sat);
  Serial.println("");
}

void SensorGPS::getCSVString(String * csvStr_p)
{
  *csvStr_p += String(state.lat); *csvStr_p += ",";
  *csvStr_p += String(state.lon); *csvStr_p += ",";
  *csvStr_p += String(state.hdop); *csvStr_p += ",";
  *csvStr_p += String(state.num_sat);
}

void SensorGPS::updateState(void)
{
  gps.get_position(&state.lat, &state.lon, &state.age);
  state.hdop = gps.hdop()*1.0/100;
  state.num_sat = gps.satellites();
}