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

### PathController

The `PathController` class defines an object that handles switching the waypoint the robot is following. Upon initialization, the object reads in a trajectory from a file called "traj.txt" on the SD card. This file is written such that each line defines a waypoint, in the format:

```
latitude,longitude,orientation
```

The `control` function takes in the current state, and a pointer to the desired waypoint. It updates the desired waypoint if the the distance between the robot and the waypoint is less than a certain success radius.

### VelocityController

The VelocityController's `control` function takes in the current state and desired waypoint, and from them, computes the necessary linear and rotational velocities using P control on the distance error, the error between the heading of the robot and the heading needed to move toward the goal position, and the error between the heading of the robot and the desired heading at the waypoint.

It implements the [controller described on the HMC 190Q website](https://www.hmc.edu/lair/E190Q/E190Q-Lecture04-PointTracking.pdf). 

### MotorController

The `control` function in the MotorController takes in the current state and the desired velocities as commanded by the velocity controller. It then determines at what speed to spin the motors, and sets these speeds in the motorDriver

As of now these speeds are calculated using the assumption that the robot's terminal linear velocity is proportional to the sum of the two motor speeds, and that the robot's terminal rotational velocity is proportional to the difference of the two motor speeds, and that the time needed to achieve these terminal velocities is small. This is a completely untested assumption as of now.

### MotorDriver

The `MotorDriver` class defines an object that handles the actual driving of the robot. The public `left` and `right` integer datamembers can be set to numbers from -127 to +127, representing max reverse and forward speeds respectively.

The `apply` function determins what PWM signals to send to which pins based upon these values, and sets up the PWM module to send the appropriate signals.

### Params.h

This file `#define`s several constants which are used throughout the codebase.