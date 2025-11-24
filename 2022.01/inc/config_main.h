/*-----------------------------------------------------------------------------
 Copyright (C) 2022-2023 University of Bologna, Italy. 
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

 File:    app_main.h                                                              
 Author:  Lorenzo Lamberti      <lorenzo.lamberti@unibo.it>                           
 Date:    20.09.2022                                                           
-------------------------------------------------------------------------------*/


// Flight
#define FORWARD_VELOCITY      0.0f      // Max forward speed [m/s].  Default: 1.0f
#define TARGET_H		          0.50f     // Target height for drone's flight [m].  Default: 0.5f

// LANDING
#define FINAL_LANDING_HEIGHT  0.07f     // [m] --> the drone drops at 0.07m of height

// SPINNING
#define SPIN_TIME             1500.0    // [ms]
#define SPIN_YAW_RATE         90.0      // [deg/s]
#define SPIN_ANGLE 	          180.0     // [deg]
#define RANDOM_SPIN_ANGLE     90.0      // [deg] add randomness to SPIN_ANGLE +/- RANDOM_SPIN_ANGLE
