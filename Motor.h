/*
 * Motor.h
 *
 *  Created on: Jun 15, 2018
 *      Author: Thanh Tu
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include "Arduino.h"
#include "Variables.h"
#include "PID_v1.h"
#include <TimerOne.h>
struct Encoder
	{
		volatile long Velocity;
		volatile long Pre_Velocity;
		volatile long Position;
	};
struct PIN
	{
		int _encodA;
		int _encodB;
		int _MotorEnable_1;
		int _MotorEnable_2;
		int _PWM_Pin;
	};
class Motor {
public:
	Motor();
	virtual ~Motor();
	void Compute_PID(PID);
	void Calculate_Speed();
	void Interrupt();
	void Limit_Speed(double,double);
	void TickPerRevolution();
	void Set_Speed(double);
	void Begin(int,int,int,int,int,bool);

  double Speed(double,double);
  double Convert_Speed_To_TickPerSecond(double);
  void Direction(double);
  void SetUpPID(PID);
	Encoder encoder;
	double Acc_Limit = 0;
	double MAX_Tick_Per_Rev = 0;

	int current_speed;
	double input = 0, output = 0, setpoint = 0;
private:

	PIN Pin;
	bool _motor;
	double angular_vel;

};

#endif /* MOTOR_H_ */
