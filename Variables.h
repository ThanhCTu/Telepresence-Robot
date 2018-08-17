/*
   Variables.h

    Created on: Jun 16, 2018
        Author: Thanh Tu
*/

#ifndef VARIABLES_H_
#define VARIABLES_H_

#define DEBUG 0

//Motor A
#define encodPinA1        2
#define encodPinB1        7
#define Mo_A1             4
#define Mo_A2             5
#define PWM_Control_Mo_A  6

//Motor B
#define encodPinA2        3
#define encodPinB2        8
#define Mo_B1             9
#define Mo_B2             10
#define PWM_Control_Mo_B  11

//Motor control variables
#define MAX_LINEAR_VEL_FW 0.8  // max forward speed +1.2m/s
#define MAX_LINEAR_VEL_BW -0.6  // max backward speed -0.6m/s
#define MAX_ANGULAR_VEL   1 // +/-1 rad/s
#define LINEAR_VEL_INC_FW 0.05
#define LINEAR_VEL_INC_BW 0.05
#define ANGULAR_VEL_INC   0.05

// Body params
#define WHEEL_RADIUS      0.0762 // 3" = 0.0762 meter
#define WHEELS_DISTANCE   0.2794 // 11" = 0.2794 meter

// Timing const
#define TICK_PER_REV      250
#define TIMER_INTERVAL    160000
#define ONE_SECOND        1000000   //1s = 1,000,000 us100

// Motors' PID params
#define KP                1
#define KI                0.01
#define KD                0.11

#define KP_small          2
#define KI_small          0.01
#define KD_small          0.06
#endif /* VARIABLES_H_ */
