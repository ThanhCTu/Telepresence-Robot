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
struct Command
{
  int PreComm;
  int Counter;
};
class Motor 
{
public:
	Motor();
	virtual ~Motor();
	void Compute_PID(PID);
	//void Calculate_Speed(PID,Motor*);
  void SetOutputToPWM(PID);
  //void Calculate_Speed();
	void Interrupt();
	void Limit_Speed(double,double);
	void TickPerRevolution();
	void Write_PWM(double);
	void Begin(int,int,int,int,int,bool);
  double Speed(double);
  double Convert_Speed_To_TickPerSecond(double);
  void Direction(double);
  void SetUpPID(PID &myPID);

  void CheckMotors(Motor*);

  void DifferentialDrive (double,int);
  //Movements
  bool bitLocation(int,byte);
  void GoStraight(int);
  void GoBack(int);
  void TurnRight(int);
  void TurnLeft(int);
  void Movement(int);
	Encoder encoder;
  Command cmd;
	double Acc_Limit = 0;
	double MAX_Tick_Per_Rev = 0;
	int current_speed;
	double input = 0, output = 0, setpoint = 0;
  static long encoder_diff;
  volatile long local_speed;
private:

	PIN Pin;
	bool _motor;
	double angular_vel;

};

#endif /* MOTOR_H_ */
