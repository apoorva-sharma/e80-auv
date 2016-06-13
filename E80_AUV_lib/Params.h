
/*
 * File:   Params.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#ifndef __PARAMS_H__
#define __PARAMS_H__

// linear velocity = sum of motor values / LIN_VEL_CONST
#define LIN_VEL_CONST 508.0

// rotational velocity = difference of motor values / ROT_VEL_CONST
#define ROT_VEL_CONST 40.4

#define MAX_LIN_VEL 0.5
#define MAX_ROT_VEL 1.0


#define MAX_NUM_WAYPOINTS 20
#define SUCCESS_RADIUS_SQUARED 0.25 // success radius in meters, squared


/* TIMING */
// how long the GPS data transfer function should take, in milliseconds
#define GPS_READ_INTERVAL 30


/* PINS */
#define SD_CHIP_SELECT 10

#endif