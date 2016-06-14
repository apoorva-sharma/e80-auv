
/*
 * File:   MotorController.cpp
 * Author: Apoorva Sharma (asharma@hmc.edu)
 *
 * Created on 8 June 2016
 */

 #include "MotorController.h"
 #include "Params.h"

 void MotorController::control(StateEstimator * stateEstimator_p, velocity_setpoint_t * desiredVelocity_p, MotorDriver * driver_p)
 {
 	int sum = (int) LIN_VEL_CONST*desiredVelocity_p->v;
 	int diff = (int) ROT_VEL_CONST*desiredVelocity_p->w;

 	driver_p->right = (sum+diff)>>1; // sum/2 + diff/2
 	driver_p->left = (sum-diff)>>1; // sum/2 - diff/2
 }