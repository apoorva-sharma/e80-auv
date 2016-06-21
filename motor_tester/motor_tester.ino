/************
 *  Motor tester
 *  Author: Apoorva Sharma (asharma@hmc.edu)
 *  Created: 20 Jun 2016
 *  Runs the motors at various speeds, and records speeds as well as IMU data
 ************/

/* Libraries */
// Adafruit 9DOF IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <SensorIMU.h>

#include <TinyGPS.h>

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
unsigned long last_loop = 0;
unsigned long last_log = 0;
#define LOOP_INTERVAL 100
#define LOG_INTERVAL 500

// Sensors
//IMU
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified();
SensorIMU imu(&dof, &accel, &mag, &gyro);

// Logger
SdFat sd;
SdFile file;
Logger logger(sd,file);

MotorDriver motorDriver(MOTOR_L_FORWARD,MOTOR_L_REVERSE,MOTOR_R_FORWARD,MOTOR_R_REVERSE); 

// sequence
#define SEQ_LEN 11
size_t curridx = 0;
int seq_l[SEQ_LEN] = {0,25,0,50,0,75,0,100,0,125,0}; 
int seq_r[SEQ_LEN] = {0,25,0,50,0,75,0,100,0,125,0};
int seq_t[SEQ_LEN] = {1,3,1,3,1,3,1,3,1,3,1};
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


  /* Initialize the Logger */
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
}

/**************************************************************************/
void loop() {
  unsigned long current_time = micros();
  
  if (current_time - last_loop >= LOOP_INTERVAL*1000) {
    last_loop = current_time;
    
    // handle state transitions
    if (current_time - last_trans >= seq_t[curridx]*1000*1000) {
      Serial.print("switching to state "); Serial.println(curridx);
      if (curridx < SEQ_LEN - 1) {
        curridx++;
        last_trans = current_time;
      }
    }

    // Gather data from serial sensors
    imu.read(); // this is a sequence of blocking I2C read calls
  
    motorDriver.left = seq_l[curridx];
    motorDriver.right = seq_r[curridx];
    motorDriver.apply();
    
    // Log at every LOG_INTERVAL
    if (current_time - last_log >= LOG_INTERVAL*1000) {
      last_log = current_time;
  
      unsigned long time_before_log = millis();
      logger.log(current_time); // this a blocking sequence of comamnds sent over SPI
      unsigned long time_after_log = millis();
      Serial.print("Time taken to log row: "); 
      Serial.println(time_after_log - time_before_log);
    }
  }
}
