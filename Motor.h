/*
   Motor.h

    Created on: Jun 15, 2018
        Author: Thanh Tu
*/

#ifndef MOTOR_H_
#define MOTOR_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Variables.h"
#include "PID_v1.h"

struct Encoder {
//  volatile long velocity;
  volatile long pos;
};

struct Pin {
  int encodA;
  int encodB;
  int MotorEnable_1;
  int MotorEnable_2;
  int PWM_Pin;
};

class Motor {
  
  public:
    Motor();
    ~Motor();
    void SetPins(int encoder_A, int encoder_B, int motor_enable_1, int motor_enable_2, int PWM_pin, boolean right);
    void EncoderInterrupt();
    int SpeedToPWM(double, boolean);
    void SetPWM(int, int);
    
    void Stop();

  private:
    PID _pid;
    Pin _pins;
    double _input = 0, _output = 0, _setpoint = 0;
    bool _dir;
    int _pwm;
    Encoder _encoder;

    void SetDirection(int);
    int SpeedToTick(double);
    
};

#endif /* MOTOR_H_ */
