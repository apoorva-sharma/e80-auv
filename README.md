# e80-auv
Arduino project for the Teensy microcontroller used for the HMC E80 AUV

Copy E80_AUV_lib to the Arduino libraries folder to build the main auv_controller.ino file.


## Main Classes

### Logger

The `Logger` class handles all the logging. In the main program's setup function, the `Logger` object's `include` function is run, passing pointers of any `DataSource` values that are to be logged. Next, the `Logger` object's `init` function is run, which automatically finds a unused filename for the log file, and opens the file, writing the column headings.

Finally the `log` function calls each `include`d `DataSource` object's `getCSVString` function, and writes the resulting row of data to the SD Card.


### DataSource

`DataSource` is an abstract class representing a source of data to be logged. `DataSource` objects must have a `headingStr` String member which contains the names of the columns, as well as a `getCSVString` function which outputs the current data values into a CSV string.

### SensorGPS, SensorIMU

These classes are wrappers for the IMU and GPS sensors, offering functions to poll the sensors for new data, and keeping internal state variables holding the latest received data. These classes implement the DataSource interface.

### StateEstimator

The `StateEstimator` class defines an object which handles all the state estimation of the robot. It keeps an internal `state` struct which stores the current estimate of the state. It is `init`ialized with the loop period (in seconds) and the origin gps coordinates (in 10^-5 deg).

It includes functions to incorporate data from the IMU and GPS, as well as a function which takes in the current commanded motor drive and applies an forward kinematic model to determine the state of the robot after one loop period.

It also implements the `DataSource` interface to plug into the `Logger` framework.

