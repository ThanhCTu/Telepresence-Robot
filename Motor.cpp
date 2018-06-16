/*
 * Motor.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Thanh Tu
 */

#include "Motor.h"

Motor::Motor() {
	// TODO Auto-generated constructor stub
	//Initialize struct
	encoder = {0,0,0};
	Pin = {0,0,0,0,0};

}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

void Motor::Begin(int encodA,int encodB, int MotorEnable_1, int MotorEnable_2, int PWM_Pin, bool motor)
{
  pinMode(encodA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodB, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(MotorEnable_1, OUTPUT);
  pinMode(MotorEnable_2, OUTPUT);
  pinMode(PWM_Pin, OUTPUT);

  //Save local variables
  Pin._encodA = encodA;
  Pin._encodB = encodB;
  Pin._MotorEnable_1 = MotorEnable_1;
  Pin._MotorEnable_2 = MotorEnable_2;
  Pin._PWM_Pin = PWM_Pin;
  _motor = motor;

  TickPerRevolution();
  setpoint = Tick_Per_Rev;
}


void Motor::TickPerRevolution()
{
  //w = v/r
  //Calculate angular velocity
  angular_vel = (double) MAX_SPEED / WHEEL_RADIUS;      //Rad/s
  //Convert to revolution per second
  angular_vel = (double) angular_vel / (2*PI);          //Rev/s
  //Tick per Second
  Tick_Per_Rev = (double) angular_vel * TICK_PER_REVOLUTION;     // Per second

  Tick_Per_Rev = (double) Tick_Per_Rev * TIME_FRAME / ONE_SECOND;      // TIME_FRAME in us => Multi with 1000 to convert to micro second
  //Limit Acceleration
  //Acc_Limit = (double) ACC_MAX  * 1000 * TICK_PER_REVOLUTION / (WHEEL_RADIUS * 2 * PI * 1000000) ;
  Acc_Limit = (double) Tick_Per_Rev / 3;
}

void Motor::Limit_Speed(double Final_Vel, double Init_Vel)
{
  //Invert Acc_Limit if the TR goes backward
  //if(Final_Vel < 0 && Init_Vel) < 0) Acc_Limit = Acc_Limit * (-1);
  //else Acc_Limit = abs(Motor::Acc_Limit);
  if(Final_Vel - Init_Vel > Acc_Limit)
      encoder.Velocity = Init_Vel + Acc_Limit;
}

void Motor::Interrupt()
{
  //double Motor::counter = 0;
  if (digitalRead(Pin._encodB) == HIGH)             encoder.Position++;           // if (digitalRead(encodPinB1)==HIGH)  count ++; (if PINB1 &0b00000001)
  else                                          	encoder.Position--;             // if (digitalRead(encodPinB1)==LOW)   count --;
}

void Motor::Calculate_Speed()
{

	encoder.Velocity = (double) encoder.Position;   //in us
	Limit_Speed(encoder.Velocity,encoder.Pre_Velocity);
	encoder.Pre_Velocity = encoder.Velocity;
	Set_Speed(output);

	//Reset Position
	encoder.Position = 0;

}
void Motor::Set_Speed(double Calculated_PID)
{
   //LImit Calculated PID
  current_speed = (int) current_speed + Calculated_PID;
  if(current_speed >= 255)
    current_speed = 255;
  else if (current_speed < 0)
    current_speed = 0;
  //Write pwm to PWM pin
  analogWrite(Pin._PWM_Pin,current_speed);

}

void Motor::Compute_PID(PID myPID)
{
  Limit_Speed(encoder.Velocity,encoder.Pre_Velocity);
  input = encoder.Velocity;
  myPID.Compute();
}

