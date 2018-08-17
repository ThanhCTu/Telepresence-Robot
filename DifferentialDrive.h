/*
   DifferentialDrive.h

    Created on: Jul 20, 2018
        Author: Thanh Hoang
*/

#ifndef DDRIVE_H_
#define DDRIVE_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Variables.h"
#include "Motor.h"

class DDrive {

  public:
    DDrive();
    void SetRightMotorPins(int, int, int, int, int);
    void SetLeftMotorPins(int, int, int, int, int);
    void RightMotorInterrupt();
    void LeftMotorInterrupt();
    void TimerInterrupt();
    void ParseCommand(unsigned char);

  private:
    Motor _motor_left;
    Motor _motor_right;
    double _linear = 0, _angular = 0;
    double _left_speed = 0, _right_speed = 0;
    int _time_cnt = 0, _r_pwm = 0, _l_pwm = 0;
    boolean isReceived = false;

    void ComputeDD(); // compute left and right velocity based on differential drive formula
};

#endif /* DDRIVE_H_ */

