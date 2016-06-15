
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

 	int rval = (sum+diff)>>1; // sum/2 + diff/2
 	int lval = (sum-diff)>>1; // sum/2 - diff/2
 	driver_p->right = (rval > 127) ? 127 : ( (rval < -127) ? -127 : rval ); 
 	driver_p->left = (lval > 127) ? 127 : ( (lval < -127) ? -127 : lval ); 
 }