/*
 * Variables.h
 *
 *  Created on: Jun 16, 2018
 *      Author: Thanh Tu
 */

#ifndef VARIABLES_H_
#define VARIABLES_H_
//Motor A
#define encodPinA1        2
#define encodPinB1        7
#define Mo_A1             4
#define Mo_A2             5
#define PWM_Control_Mo_A  6
#define Motor_Right       true
//Motor B
#define Motor_Left        false
#define encodPinA2        3
#define encodPinB2        8
#define Mo_B1             9
#define Mo_B2             10
#define PWM_Control_Mo_B  11
#define Motor_Left       false

//Bits for forward,backward,right,left
#define FORWARD_BIT       3
#define BACKWARD_BIT      2
#define RIGHT_BIT         1
#define LEFT_BIT          0
#define STOPPING          0
//Const variables
#define ACC_MAX           100
#define MAX_SPEED         1000
#define WHEEL_RADIUS      12
#define SET_SPEED         1.0//WHEEL_RADIUS  //rps
#define TICK_PER_REVOLUTION 250
#define TIME_FRAME       50000                     //in us ( 100,000 us = 100 ms)
#define KP                1     // 0.1
#define KI                0
#define KD                20    //2
#define ONE_SECOND       1000000   //1s = 1,000,000 us
#define LENGTH_OF_TWO_WHEELS    50 //units : centimeter  (the length of two wheels)
//const double Max_Speed = 0.8;                              //Max Speed = 3 m/s => 3 puls/ms
#define BACKWARD_LIMIT    -0.7
struct BUS
{
  int counter;
  bool curr_state;
  bool prev_state;
};

#endif /* VARIABLES_H_ */
