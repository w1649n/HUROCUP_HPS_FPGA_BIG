/*
 * main.h
 *
 *  Created on: 2019/09/11
 *      Author: Yu-Chih, Wang
 */

#ifndef MAIN_H_
#define MAIN_H_

/******************* Define******************************/
//#define main_debug    /* Turn debugging on */
/********************************************************/

/******************* Parameter **************************/

/********************************************************/

/******************* Include libarary*********************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include <iostream>
#include <Eigen/Eigen>
/********************************************************/

/******************* Include module**********************/
#include "hps_0.h"
#include "Initial.h"
#include "Inverse_kinematic.h"
#include "Walkinggait.h"
#include "Sensor.h"
#include "hand_kinetic_base.h"
#include "IMU_base_obs.h"
#include "Feedback_Motor.h"
//#include "inc\WalkingGait.h"
/********************************************************/

Feedback_Motor feedbackmotor;
Locus locus;
InverseKinematic IK;
Initial init;
Walkinggait walkinggait;
Datamodule datamodule;
SensorDataProcess sensor;
BalanceControl balance;
Hand_InverseKinematic HandIK;
IMU_base_obs IB;
WalkingGaitByLIPM walkingGaitByLIPM;

struct timeval tstart, tend;
double timeuse;
Vector3d WpA_,BpA_,Theta_;

#endif /* MAIN_H_ */
