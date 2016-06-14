/************
 *  E80 AUV controller
 *  Author: Apoorva Sharma (asharma@hmc.edu)
 *  Created: 6 Jun 2016
 ************/

/* Libraries */
// GPS
#include <TinyGPS.h>
#include <SensorGPS.h>

// Adafruit 9DOF IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <SensorIMU.h>

// State Estimator and Controllers
#include <StateEstimator.h>
#include <PathController.h>
#include <VelocityController.h>
#include <MotorDriver.h>
#include <MotorController.h>

// Logging
#include <SD.h>
#include <SPI.h>
#include <Logger.h>


#include <Params.h>



/* Global Variables */
// Timing
unsigned long last_loop = 0;
unsigned long last_log = 0;
#define LOOP_INTERVAL 100
#define LOG_INTERVAL 1000

// Sensors
// IMU
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified();
SensorIMU imu(&dof, &accel, &mag, &gyro);

//GPS
HardwareSerial Uart = HardwareSerial(); // pin 1 = rx (from gps tx), pin 2 = tx (from gps rx)
SensorGPS gps(&Uart);

// Logger
Logger logger;

// State Estimation
StateEstimator stateEstimator;

// Trajectory following controllers
PathController pathController;
VelocityController velocityController;
MotorDriver motorDriver(1,2,3,4); //TODO replace with real pins
MotorController motorController;

waypoint_t desiredPosition;

velocity_setpoint_t desiredVelocities;


/**************************************************************************/
void setup() {
  Serial.begin(115200);
  delay(2000); // Wait to ensure computer monitor is ready
  Serial.println(F("Serial connection started")); Serial.println("");

  Serial.print("\nLogger: Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("Card initialized.");

  // Determine GPS origin
  // 34.103835, -117.708172 is the center of the circle in scripps pool
  float lat = 34.10383;
  float lon = -117.70817;
  
  /* init the stateEstimator with an origin lat/lon */
  stateEstimator.init(0.1,lat,lon);

  /* init the pathController */
  pathController.init("traj.txt", &stateEstimator, &desiredPosition);

  /* Initialize the Logger */
  logger.include(&gps);
  logger.include(&imu);
  logger.include(&stateEstimator);
  logger.init();
  
  /* Initialise the sensors */
  gps.init();
  imu.init();
}

/**************************************************************************/
void loop() {
  unsigned long current_time = micros();
  
  if (current_time - last_loop >= LOOP_INTERVAL*1000) {
    last_loop = current_time;
    
    Serial.print("/----");
    Serial.println(current_time);  
  
    bool newGPSData;
    bool newIMUData;

    // Gather data from serial sensors
    //newIMUData = imu.read(); // this is a sequence of blocking I2C read calls
    //newGPSData = gps.read(); // this is a sequence of UART reads, bounded by a time
  
    // Use Data
    if (newIMUData) {
      //Serial.print(" ");
      //imu.printState();
      //stateEstimator.incorporateIMU(&imu.state);
    }

    if (newGPSData) {
      //Serial.print(" ");
      //gps.printState();
      //stateEstimator.incorporateGPS(&gps.state);
    }

    // Print current state estimate
    //stateEstimator.printState();

    // Controllers
    pathController.control(&stateEstimator, &desiredPosition);
    velocityController.control(&stateEstimator, &desiredPosition, &desiredVelocities);
    motorController.control(&stateEstimator, &desiredVelocities, &motorDriver);

    //motorDriver.apply();
    stateEstimator.incorporateControl(&motorDriver);
    
    // Log at every LOG_INTERVAL
    if (current_time - last_log >= LOG_INTERVAL*1000) {
      last_log = current_time;
  
      unsigned long time_before_log = millis();
      logger.log(current_time); // this a blocking sequence of comamnds sent over SPI
      unsigned long time_after_log = millis();
      Serial.print("Time taken to log row: "); 
      Serial.println(time_after_log - time_before_log);
    }
    
    Serial.println(micros()-current_time);
    Serial.println("\\----");
  }
}
