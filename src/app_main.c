/*-----------------------------------------------------------------------------
 Copyright (C) 2021-2022 University of Bologna, Italy, ETH Zurich, Switzerland. 
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
 Author:  Vlad Niculescu      	<vladn@iis.ee.ethz.ch>                           
 Date:    18.02.2022                                                           
-------------------------------------------------------------------------------*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

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
#include <math.h>
#include "config_main.h"


/* --------------- GUI PARAMETERS --------------- */
// Global variables for the parameters
float forward_vel = FORWARD_VELOCITY;
float flying_height = TARGET_H;

// My parameters for enabling/disabling some parts ofss code. 1=Active, 0=Non active
uint8_t debug = 1; 		// activate debug prints

// START / STOP mission parameter
uint8_t fly = 0; 		// Takeoff/landing command (GUI parameter)
uint8_t landed = 0; 	// Flag for indicating whether the drone landed

/* --------------- GLOBAL VARIABLES --------------- */

setpoint_t setp_dronet;

/* --------------- FUNCTION DEFINITION --------------- */ 
void land(void);
void takeoff(float height);
void headToPosition(float x, float y, float z, float yaw);
static setpoint_t create_setpoint(float x_vel, float z, float yaw_rate);
// float low_pass_filtering(float data_new, float data_old, float alpha);

/* --------------- FUNCTIONS --------------- */ 
// Fly forward functions
static setpoint_t create_setpoint(float x_vel, float z, float yaw_rate)
{
	setpoint_t setpoint;	
	memset(&setpoint, 0, sizeof(setpoint_t));
	setpoint.mode.x = modeVelocity;
	setpoint.mode.y = modeVelocity;
	setpoint.mode.z = modeAbs;
	setpoint.mode.yaw = modeVelocity;

	setpoint.velocity.x	= x_vel;
	setpoint.velocity.y	= 0.0f;
	setpoint.position.z = z;
	setpoint.attitudeRate.yaw = yaw_rate;
	setpoint.velocity_body = true;
	return setpoint;
}

void headToPosition(float x, float y, float z, float yaw)
{
	setpoint_t setpoint;
	memset(&setpoint, 0, sizeof(setpoint_t));

	setpoint.mode.x = modeAbs;
	setpoint.mode.y = modeAbs;
	setpoint.mode.z = modeAbs;
	setpoint.mode.yaw = modeAbs;

	setpoint.position.x = x;
	setpoint.position.y = y;
	setpoint.position.z = z;
	setpoint.attitude.yaw = yaw;
	commanderSetSetpoint(&setpoint, 3);
}

// TAKEOFF and LANDING FUNCTIONS
void takeoff(float height)
{
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
	for(int i=(int)100*height; i>5; i--)
	{
		headToPosition(pos.x, pos.y, (float)i / 100.0f, 0);
		vTaskDelay(20);
	}
	vTaskDelay(200);
}

// float low_pass_filtering(float data_new, float data_old, float alpha)
// {
// 	float output;
// 	// Low pass filter the forward velocity
// 	output = (1.0f - alpha) * data_new + alpha * data_old;
// 	return output;
// }

void appMain()
{
	DEBUG_PRINT("Dronet v2 started! \n");
	systemWaitStart();
	vTaskDelay(1000);
	/* ------------------------- NOT FLYING ------------------------- */

	while(!fly)
	{
		if (debug==1) DEBUG_PRINT("Waiting start \n");			
		vTaskDelay(100);
	}

	/* ------------------------- TAKING OFF ------------------------- */

	// reset the estimator before taking off	
	estimatorKalmanInit();  
	// TAKE OFF
	takeoff(flying_height);	

	/* ------------------------ Flight Loop ------------------------ */

	while(1) {
		vTaskDelay(5);
		if (fly==0 && landed==0)//land
		{
			land();
			landed=1;
		}	
		if (fly==1 && landed==1) //start flying again
		{
			estimatorKalmanInit();  
			takeoff(flying_height);				
			landed=0;
		}			
		if (debug==1) DEBUG_PRINT("flying\n");			

		// Give setpoint to the controller
		if (fly==1){
			setp_dronet = create_setpoint(forward_vel, flying_height, 0.0);
			commanderSetSetpoint(&setp_dronet, 3);
		}
		
		vTaskDelay(30);
	}
}


/* --- TIP for Logging or parameters --- */
// The variable name: PARAM_ADD(TYPE, NAME, ADDRESS)
// both for logging (LOG_GROUP_START) or for parameters (PARAM_GROUP_START) 
// should never exceed 9 CHARACTERS, otherwise the firmware won't start correctly

/* --- PARAMETERS --- */ 
PARAM_GROUP_START(START_STOP)
	PARAM_ADD(PARAM_UINT8, fly, &fly)
PARAM_GROUP_STOP(DRONET_PARAM)

// Activate - deactivate functionalities: 0=Non-active, 1=active
PARAM_GROUP_START(FUNCTIONALITIES)
	PARAM_ADD(PARAM_UINT8, debug, &debug) // debug prints
PARAM_GROUP_STOP(DRONET_SETTINGS)

// Filters' parameters
PARAM_GROUP_START(DRONET_PARAMS)
	PARAM_ADD(PARAM_FLOAT, velocity, &forward_vel)
	PARAM_ADD(PARAM_FLOAT, height, &flying_height)
PARAM_GROUP_STOP(DRONET_SETTINGS)




