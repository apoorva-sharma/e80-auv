/************
 *  Motor Controller Tester
 *  Author: Apoorva Sharma (asharma@hmc.edu)
 *  Created: 27 Jun 2016
 *  Gives a sequence of velocity setpoints, which the motorController
 *  must try and meet.
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
// HardwareSerial Uart = HardwareSerial(); // pin 1 = rx (from gps tx), pin 2 = tx (from gps rx)
// SensorGPS gps(&Uart);

// Logger
SdFat sd;
SdFile file;
Logger logger(sd,file);

StateEstimator stateEstimator;
MotorDriver motorDriver(MOTOR_L_FORWARD,MOTOR_L_REVERSE,MOTOR_R_FORWARD,MOTOR_R_REVERSE); 
MotorController motorController;

velocity_setpoint_t desiredVelocities;

// sequence
#define SEQ_LEN 11
size_t curridx = 0;
float seq_v[SEQ_LEN] = {0,0.1,0,0.25,0,0.5,0,0.75,0,1.0,0}; 
float seq_w[SEQ_LEN] = {0,0,0,0,0,0,0,0,0,0,0};
int seq_t[SEQ_LEN] = {60,5,20,5,20,5,20,5,20,5,20};
unsigned long last_trans = 0;


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

  logger.include(&stateEstimator);
  logger.include(&imu);
  logger.include(&motorDriver);
  logger.init();
  
  /* Initialise the sensors */
  imu.init();

  /* Initialize the motor pins */
  pinMode(MOTOR_L_FORWARD,OUTPUT);
  pinMode(MOTOR_L_REVERSE,OUTPUT);
  pinMode(MOTOR_R_FORWARD,OUTPUT);
  pinMode(MOTOR_R_REVERSE,OUTPUT);

  Serial.println("starting control loop");
  controlTimer.begin(controlLoop, LOOP_INTERVAL*1000);
}

/**************************************************************************/
void controlLoop(void) {
  unsigned long current_time = millis();
    
  // handle state transitions
  if (current_time - last_trans >= seq_t[curridx]*1000) {
    if (curridx < SEQ_LEN - 1) {
      curridx++;
      last_trans = current_time;
    }
    Serial.print("switching to state "); Serial.println(curridx);
  }

  // Gather data from serial sensors
  imu.read(); // this is a sequence of blocking I2C read calls

  desiredVelocities.v = seq_v[curridx];
  desiredVelocities.w = seq_w[curridx];

  motorController.control(&stateEstimator, &desiredVelocities, &motorDriver);
  motorDriver.apply();
  stateEstimator.incorporateControl(&motorDriver);

  motorDriver.printState();
  stateEstimator.printState();

  logger.log(current_time); 
}

/**************************************************************************/
void loop() {
  // only work on writing the files
  logger.write();
}
