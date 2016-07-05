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
// control loop interval in ms
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
  double lat = 34.103835;
  double lon = -117.708172;
  
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
  unsigned long t0 = micros();

  bool newGPSData;
  bool newIMUData;

  // Gather data from serial sensors
  newIMUData = imu.read(); // this is a sequence of blocking I2C read calls
  unsigned long t1 = micros();

  newGPSData = gps.read(); // this is a sequence of UART reads, bounded by a time
  unsigned long t2 = micros();

  // Use Data
  if (newIMUData) {
    stateEstimator.incorporateIMU(&imu.state);
  }
  unsigned long t3 = micros();


  if (newGPSData) {
    stateEstimator.incorporateGPS(&gps.state);
  }
  unsigned long t4 = micros();

  // Controllers
  pathController.control(&stateEstimator, &desiredPosition);
  velocityController.control(&stateEstimator, &desiredPosition, &desiredVelocities);
  motorController.control(&stateEstimator, &desiredVelocities, &motorDriver);
  stateEstimator.incorporateControl(&motorDriver);
  
  motorDriver.apply();
  unsigned long t5 = micros();

  logger.log(t0); 

  unsigned long t6 = micros();

  // Print things
  // Print current state estimate
  stateEstimator.printState();
  Serial.print("desired v:");
  Serial.print(desiredVelocities.v);
  Serial.print(" w:");
  Serial.println(desiredVelocities.w);
  motorDriver.printState();

  Serial.print("IMU read time (us): "); Serial.println(t1-t0);
  Serial.print("GPS read time (us): "); Serial.println(t2-t1);
  Serial.print("  IMU incorp. time: "); Serial.println(t3-t2);
  Serial.print("  GPS incorp. time: "); Serial.println(t4-t3);
  Serial.print("Control calc. time: "); Serial.println(t5-t4);
  Serial.print("Log to buffer time: "); Serial.println(t6-t5);
  Serial.print("   Total time (us): "); Serial.println(t6-t0);
}


/**************************************************************************/
void loop() {
  // only work on writing the files
  logger.write();
}
