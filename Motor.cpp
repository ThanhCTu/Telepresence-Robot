/*
   Motor.cpp

    Created on: Jun 15, 2018
        Author: Thanh Tu
*/

#include "Motor.h"
#include <math.h>

Motor::Motor() {
  //Initialize struct
  _encoder = {0}; _pins = {0, 0, 0, 0, 0};
  _pid.SetPID(&_input, &_output, &_setpoint, KP, KI, KD, DIRECT);
  _pid.SetMode(AUTOMATIC);
  _pid.SetOutputLimits(-255, 255);
}

Motor::~Motor() {
  // TODO Auto-generated destructor stub
}

void Motor::SetPins(int encoder_A, int encoder_B, int motor_enable_1, int motor_enable_2, int PWM_pin, boolean right) {
  pinMode(encoder_A, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder_B, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(motor_enable_1, OUTPUT);
  pinMode(motor_enable_2, OUTPUT);
  pinMode(PWM_pin, OUTPUT);

  //Save local variables
  _pins.encodA = encoder_A;
  _pins.encodB = encoder_B;
  _pins.MotorEnable_1 = motor_enable_1;
  _pins.MotorEnable_2 = motor_enable_2;
  _pins.PWM_Pin = PWM_pin;
  _dir = right;
}

int Motor::SpeedToTick(double velocity) {
  return round(velocity * TICK_PER_REV * TIMER_INTERVAL / (2 * PI * WHEEL_RADIUS * ONE_SECOND));      //Rad/s
}

int Motor::SpeedToPWM(double velocity , boolean debug) {
  if (abs(velocity) < 0.07 * MAX_LINEAR_VEL_FW)
    _pid.SetTunings(KP_small, KI_small, KD_small);
  else
    _pid.SetTunings(KP, KI, KD);

  _setpoint = SpeedToTick(velocity);
  _input = _encoder.pos;

  _pid.Compute();

  _pwm += _output;

  // PWM boundary conditions
  _pwm = (_pwm > 255) ? 255 : _pwm;
  _pwm = (_pwm < -255) ? -255 : _pwm;

  if ((_encoder.pos + _setpoint) == 0) _pwm = 0;
  if (debug) {
    Serial.print(_setpoint); Serial.print(" "); Serial.print(_encoder.pos); Serial.print(" "); Serial.println(_pwm); //Serial.println(map(pwm,-255,255,setpoint,setpoint));
  }

  _encoder.pos = 0;
  return _pwm;
}

void Motor::SetPWM(int pwm, int kickstart = 0) {
  SetDirection(pwm);
  analogWrite(_pins.PWM_Pin, abs(pwm));
}

void Motor::Stop() {
  SetPWM(0);
}

void Motor::SetDirection(int pwm) {
  //Identify direction of the robot when receiving user's command
  if (pwm > 0) {
    digitalWrite(_pins.MotorEnable_1, HIGH);
    digitalWrite(_pins.MotorEnable_2, LOW);
  }
  else {
    digitalWrite(_pins.MotorEnable_1, LOW);
    digitalWrite(_pins.MotorEnable_2, HIGH);
  }
}

void Motor::EncoderInterrupt() {
  if (digitalRead(_pins.encodB) == HIGH) {
    if (_dir) _encoder.pos--;
    else _encoder.pos++;
  }
  else {
    if (_dir) _encoder.pos++;
    else _encoder.pos--;
  }
}

