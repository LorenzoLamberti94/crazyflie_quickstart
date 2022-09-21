/*-----------------------------------------------------------------------------
 Copyright (C) 2022-2023 University of Bologna, Italy, ETH Zurich, Switzerland. 
 All rights reserved.                                                           
                                                                               
 Licensed under the Apache License, Version 2.0 (the "License");               
 you may not use this file except in compliance with the License.              
 See LICENSE.apache.md in the top directory for details.                       
 You may obtain a copy of the License at                                       
                                                                               
   http://www.apache.org/licenses/LICENSE-2.0                                  
                                                                               
 Unless required by applicable law or agreed to in writing, software           
 distributed under the License is distributed on an "AS IS" BASIS,             
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.      
 See the License for the specific language governing permissions and           
 limitations under the License.                                                
                                                                               
 File:    app_main.c                                                              
 Author:  Lorenzo Lamberti      <lorenzo.lamberti@unibo.it>                           
 		  Vlad Niculescu      	<vladn@iis.ee.ethz.ch>                           
 Date:    20.09.2022                                                           
-------------------------------------------------------------------------------*/

// TODO:
// [] 

// standard
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h> // for rand()
#include <math.h>
// CF
#include "app.h"
#include "FreeRTOS.h"
#include "system.h"
#include "task.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "commander.h"
#include "log.h"
#include "param.h"
// my headers
#include "config_main.h"
#include "app_main.h"


/* --------------- DEFINES --------------- */
#define PI 3.1415926f


/* --------------- GUI PARAMETERS --------------- */

// -- My parameters for enabling/disabling some parts of code. 
// 	  1=Active, 0=Non active
uint8_t fly = 0; 		// Takeoff/landing command (GUI parameter)
uint8_t debug = 1; 		// activate debug prints

// Global variables for the parameters
float forward_vel 	= FORWARD_VELOCITY;
float flying_height = TARGET_H;

// Manouver: Spin -- parameters
float spin_time 		= SPIN_TIME; 			// ms
float spin_angle 		= SPIN_ANGLE; 			// deg
float max_rand_angle 	= RANDOM_SPIN_ANGLE; 	// deg

// Demo: Manouvers -- 1=Active, 0=Non active
uint8_t circle = 0; 	
uint8_t spin_drone = 0; 	

/* --------------- GLOBAL VARIABLES --------------- */
// -- Flags
uint8_t landed = 0; 	// Flag for indicating whether the drone landed

/* --------------- FUNCTION DEFINITION --------------- */ 
void land(void);
void takeoff(float height);
void headToVelocity(float x_vel, float y_vel, float z_pos, float yaw_rate);
void headToPosition(float x, float y, float z, float yaw);
setpoint_t create_velocity_setpoint(float x_vel, float y_vel, float z_pos, float yaw_rate);
setpoint_t create_position_setpoint(float x, float y, float z, float yaw);
// float low_pass_filtering(float data_new, float data_old, float alpha);

/* ----------------------------------------------------------------------- */ 
/* ------------------------------ FUNCTIONS ------------------------------ */ 
/* ----------------------------------------------------------------------- */ 

/* --------------- Setpoint Utils --------------- */ 

setpoint_t fly_setpoint;
setpoint_t create_velocity_setpoint(float x_vel, float y_vel, float z_pos, float yaw_rate)
{    
	setpoint_t setpoint;
    memset(&setpoint, 0, sizeof(setpoint_t));
    setpoint.mode.x 	= modeVelocity;
    setpoint.mode.y 	= modeVelocity;
    setpoint.mode.z 	= modeAbs;
    setpoint.mode.yaw 	= modeVelocity;
    setpoint.velocity.x 	= x_vel;
    setpoint.velocity.y 	= y_vel;
    setpoint.position.z 	= z_pos;
    setpoint.attitude.yaw 	= yaw_rate;
    setpoint.velocity_body 	= true;
	return setpoint;
}

void headToVelocity(float x_vel, float y_vel, float z_pos, float yaw_rate)
{
    fly_setpoint = create_velocity_setpoint(x_vel, y_vel, z_pos, yaw_rate);
    commanderSetSetpoint(&fly_setpoint, 3);
}

setpoint_t create_position_setpoint(float x, float y, float z, float yaw)
{    
	setpoint_t setpoint;
	memset(&setpoint, 0, sizeof(setpoint_t));
	setpoint.mode.x 	= modeAbs;
	setpoint.mode.y 	= modeAbs;
	setpoint.mode.z 	= modeAbs;
	setpoint.mode.yaw 	= modeAbs;
	setpoint.position.x 	= x;
	setpoint.position.y 	= y;
	setpoint.position.z 	= z;
	setpoint.attitude.yaw 	= yaw;
	return setpoint;
}

void headToPosition(float x, float y, float z, float yaw)
{
    fly_setpoint = create_position_setpoint(x, y, z, yaw);
	commanderSetSetpoint(&fly_setpoint, 3);
}


/* --------------- Takeoff and Landing --------------- */ 

void takeoff(float height)
{
	// init Kalman estimator before taking off
	estimatorKalmanInit();  

	point_t pos;
	memset(&pos, 0, sizeof(pos));
	estimatorKalmanGetEstimatedPos(&pos);

	// first step: taking off gradually, from a starting height of 0.2 to the desired height
	int endheight = (int)(100*(height-0.2f));
	for(int i=0; i<endheight; i++)
	{
		headToPosition(pos.x, pos.y, 0.2f + (float)i / 100.0f, 0);
		vTaskDelay(50);
	}
	// keep constant height
	for(int i=0; i<100; i++)
	{
		headToPosition(pos.x, pos.y, height, 0);
		vTaskDelay(50);
	}
}


void land(void)
{
	point_t pos;
	memset(&pos, 0, sizeof(pos));
	estimatorKalmanGetEstimatedPos(&pos);

    float height = pos.z;
    float current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));
	
    for(int i=(int)100*height; i>100*FINAL_LANDING_HEIGHT; i--) {
		headToPosition(pos.x, pos.y, (float)i / 100.0f, current_yaw);
		vTaskDelay(20);
	}
	vTaskDelay(200);
}

/* --------------- Filtering-processing --------------- */ 

float low_pass_filtering(float data_new, float data_old, float alpha)
{
	float output;
	// Low pass filter the forward velocity
	output = (1.0f - alpha) * data_new + alpha * data_old;
	return output;
}

double sigmoid(float x)
{
     float result;
     result = 1 / (1 + expf(-x));
     return result;
}

void softmax(float* array, uint8_t softmax_range){
	int idx_max;
	idx_max = find_max_index(array, softmax_range);

	for(int i=0; i<softmax_range; i++){
		if (idx_max == i){
			array[i] = 1.0;
		}
		else{
			array[i] = 0.0;
		}
	}
}
/* --------------- Other Manouvers --------------- */ 

void flyCircle(float radius, float velocity){
    
	float distance = 2.0f*PI*radius;
    uint16_t steps = distance/velocity*1000/100; //one step is 100ms

	point_t pos;
	memset(&pos, 0, sizeof(pos));
	estimatorKalmanGetEstimatedPos(&pos);

    for (int i = 0; i < steps; i++) {
        float a = M_PI + i*2*M_PI/steps + 4;
        float x = (float)cos(a)*radius + radius + pos.x;
        float y = (float)sin(a)*radius + pos.y;
        headToPosition(x, y, pos.z, 0);
        vTaskDelay(100);
    }
}

void spin_in_place(float angle, float time){
	/*
	angle [deg]: given the current orientation, spin by "angle" degrees in place;
	time   [ms]: how much time to perform the entire manuever --> impacts the spinning speed;
	*/

    float current_yaw;					// fetch current yaw self estimation
    float steptime = 10; 				//ms
    float t_steps = (time/steptime); 	// angle steps
    float r_steps = (angle/t_steps); 	// time steps
	float new_yaw;						// new yaw given to the controller. This parameter is updated by the for loop

	// access self estimation
	point_t pos;
	memset(&pos, 0, sizeof(pos));
	estimatorKalmanGetEstimatedPos(&pos);
	current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));
    DEBUG_PRINT("%f\n",(double)current_yaw);

	// perform manuever
    for (int i = 0; i <= t_steps; i++) {
        new_yaw = (i*r_steps) + current_yaw;
    	DEBUG_PRINT("%f\n",(double)new_yaw);
		headToPosition(pos.x, pos.y, pos.z, new_yaw);
		vTaskDelay(M2T(steptime));
    }
}

/* ------------------------------------------------------------------------- */ 
/* ------------------------------ Flight Loop ------------------------------ */ 
/* ------------------------------------------------------------------------- */ 
void flight_loop(){

	if (circle==1){
		DEBUG_PRINT("Cicle!\n");
		flyCircle(0.5, 0.5);	
	}

	if (spin_drone==1){
		DEBUG_PRINT("SPIN IN PLACE!\n");
		spin_in_place(spin_angle, spin_time);
		spin_drone=0;
	}

	// Give setpoint to the controller
	headToVelocity(forward_vel, 0.0, flying_height, 0.0);
		
}

/* ------------------------------------------------------------------------ */ 
/* ------------------------------    Main    ------------------------------ */ 
/* ------------------------------------------------------------------------ */ 

void appMain()
{
	DEBUG_PRINT("Dronet v2 started! \n");
	systemWaitStart();
	vTaskDelay(1000);

	/* ------------------------ Main loop ------------------------ */

	while(1) {
		vTaskDelay(10);

		// landed, waiting to start
		if (fly==0 && landed==1) 
		{
			if (debug==1) DEBUG_PRINT("Waiting start \n");			
			vTaskDelay(M2T(1));
		}	

		// land
		if (fly==0 && landed==0) 
		{
			if (debug==1) DEBUG_PRINT("Landing\n");			
			land();
			landed=1;
		}	

		// take-off
		if (fly==1 && landed==1) 
		{
			if (debug==1) DEBUG_PRINT("Taking off\n");			
			takeoff(flying_height);				
			landed=0;
		}

		// flight loop
		if (fly==1){ 
			if (debug==1) DEBUG_PRINT("flying\n");			
			flight_loop();
		}
	}
}


/* -------------------------------------------------------------------------------- */ 
/* ------------------------------ Logging/Parameters ------------------------------ */ 
/* -------------------------------------------------------------------------------- */ 
/* --- TIP for Logging or parameters --- */
// The variable name: PARAM_ADD(TYPE, NAME, ADDRESS)
// both for logging (LOG_GROUP_START) or for parameters (PARAM_GROUP_START) 
// should never exceed 9 CHARACTERS, otherwise the firmware won't start correctly

/* --------------- LOGGING --------------- */ 
LOG_GROUP_START(DRONET_LOG)
	LOG_ADD(LOG_FLOAT, fwd_vel, &forward_vel)  	// forward velocity
LOG_GROUP_STOP(DRONET_LOG)

/* --------------- PARAMETERS --------------- */ 
PARAM_GROUP_START(START_STOP)
	PARAM_ADD(PARAM_UINT8, fly, &fly)
PARAM_GROUP_STOP(START_STOP)

// Activate - deactivate functionalities: 0=Non-active, 1=active
PARAM_GROUP_START(FUNCTIONALITIES)
	PARAM_ADD(PARAM_UINT8, debug, &debug) 			// debug prints
	PARAM_ADD(PARAM_UINT8, circle, &circle) 		// fly in circle
	PARAM_ADD(PARAM_UINT8, spin_dron, &spin_drone) 	// spin in place
PARAM_GROUP_STOP(FUNCTIONALITIES)

// Filters' parameters
PARAM_GROUP_START(DRONET_PARAMS)
	PARAM_ADD(PARAM_FLOAT, velocity, &forward_vel)
	PARAM_ADD(PARAM_FLOAT, height, &flying_height)
	PARAM_ADD(PARAM_FLOAT, t_spin, &spin_time)
	PARAM_ADD(PARAM_FLOAT, spin_ang, &spin_angle)
PARAM_GROUP_STOP(DRONET_SETTINGS)




