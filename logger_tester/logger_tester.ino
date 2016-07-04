/************
 *  Logger tester
 *  Author: Apoorva Sharma (asharma@hmc.edu)
 *  Created: 30 Jun 2016
 *  Doesn't run the motors, but does everything else
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
#include <SdFat.h>
#include <SPI.h>
#include <Logger.h>

#include <Params.h>


/* Global Variables */
// Timing
unsigned long last_log = 0;
#define LOOP_INTERVAL 100
IntervalTimer controlTimer;

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
SdFat sd;
SdFile file;
Logger logger(sd,file);

// State Estimation
StateEstimator stateEstimator;

// Trajectory following controllers
PathController pathController(file);
VelocityController velocityController;
MotorDriver motorDriver(MOTOR_L_FORWARD,MOTOR_L_REVERSE,MOTOR_R_FORWARD,MOTOR_R_REVERSE); 
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
  if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
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
  // desiredPosition.x = -10.0;
  // desiredPosition.y = 10.0;
  // desiredPosition.heading = 0.0;

  /* Initialize the Logger */
  logger.include(&gps);
  logger.include(&imu);
  logger.include(&stateEstimator);
  logger.include(&motorDriver);
  logger.init();
  
  /* Initialise the sensors */
  gps.init();
  Serial.println("initialized gps");
  imu.init();
  Serial.println("initialized imu");

  delay(60*1000); // delay for 1 min
  Serial.println("starting control loop");
  controlTimer.begin(controlLoop, LOOP_INTERVAL*1000);
}

/**************************************************************************/
void controlLoop(void) {
  unsigned long current_time = micros();

  bool newGPSData;
  bool newIMUData;

  // Gather data from serial sensors
  newIMUData = imu.read(); // this is a sequence of blocking I2C read calls
  newGPSData = gps.read(); // this is a sequence of UART reads, bounded by a time

  // Use Data
  if (newIMUData) {
    //stateEstimator.incorporateIMU(&imu.state);
  }

  if (newGPSData) {
    stateEstimator.incorporateGPS(&gps.state);
  }

  // Controllers
  pathController.control(&stateEstimator, &desiredPosition);
  velocityController.control(&stateEstimator, &desiredPosition, &desiredVelocities);
  motorController.control(&stateEstimator, &desiredVelocities, &motorDriver);
  stateEstimator.incorporateControl(&motorDriver);
  
  motorDriver.apply();

  logger.log(current_time); 

  unsigned long new_time = micros();
  Serial.print("ISR time (us): ");
  Serial.println(new_time - current_time);
}


/**************************************************************************/
void loop() {
  // only work on writing the files
  logger.write();
}
