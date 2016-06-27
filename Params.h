
/*
 * File:   Params.h
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

#ifndef __PARAMS_H__
#define __PARAMS_H__

/* CONTROL MODEL */
// [a b; c d]*[rpwm; lpwm] = [v; w]
#define FWD_MODEL_A 0.0006807
#define FWD_MODEL_B 0.00063245
#define FWD_MODEL_C 0.0049
#define FWD_MODEL_D -0.0107
// [a b; c d]*[v; w] = [rpwm; lpwm]
#define INV_MODEL_A 1029.0
#define INV_MODEL_B 61.0123
#define INV_MODEL_C 473.666
#define INV_MODEL_D -65.6686

// // linear velocity = sum of motor values / LIN_VEL_CONST
// #define LIN_VEL_CONST 508.0

// // rotational velocity = difference of motor values / ROT_VEL_CONST
// #define ROT_VEL_CONST 40.4

#define MAX_LIN_VEL 0.5
#define MAX_ROT_VEL 1.0


/* PATH CONTROLLER */
#define MAX_NUM_WAYPOINTS 20
#define SUCCESS_RADIUS_SQUARED 0.25 // success radius in meters, squared


/* TIMING */
// how long the GPS data transfer function should take, in milliseconds
#define GPS_READ_INTERVAL 10

/* PINS */
#define SD_CHIP_SELECT 10

#define MOTOR_L_FORWARD 3
#define MOTOR_L_REVERSE 4
#define MOTOR_R_FORWARD 22
#define MOTOR_R_REVERSE 21

/* MOTOR DYNAMICS */
// the minimum PWM amount that causes the motors to actually spin
#define MOTOR_L_DEADZONE 32
#define MOTOR_R_DEADZONE 34 
// factors (0 < x <= 1) applied to the powers given to the motors
// to correct for imbalance
#define MOTOR_L_FACTOR 0.541
#define MOTOR_R_FACTOR 1.0

#endif