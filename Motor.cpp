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

  setpoint = MAX_Tick_Per_Rev;

}


void Motor::TickPerRevolution()
{


  MAX_Tick_Per_Rev = Convert_Speed_To_TickPerSecond(MAX_SPEED);

  //Limit Acceleration
  Acc_Limit = Convert_Speed_To_TickPerSecond(ACC_MAX);

  Serial.print("TickperRev: ");
  Serial.println(MAX_Tick_Per_Rev);


}

void Motor::Limit_Speed(double Final_Vel, double Init_Vel)
{

  //Invert Acc_Limit if the TR goes backward
  if(Final_Vel - Init_Vel > Acc_Limit)
        encoder.Velocity = Init_Vel + Acc_Limit;
}

void Motor::Interrupt()
{

  //double Motor::counter = 0;

    if (digitalRead(Pin._encodB) == HIGH)             encoder.Position++;           // if (digitalRead(encodPinB1)==HIGH)  count ++; (if PINB1 &0b00000001)
    else                                          	  encoder.Position--;             // if (digitalRead(encodPinB1)==LOW)   count --;

}

void Motor::Calculate_Speed()
{
  //Calculate velocity (v = s/t)
	encoder.Velocity = (double) abs(encoder.Position);   //in us
	Limit_Speed(encoder.Velocity,encoder.Pre_Velocity);
	encoder.Pre_Velocity = encoder.Velocity;
  //Set the PWM

	Set_Speed(output);
  

	//Reset Position
	encoder.Position = 0;

}
void Motor::Set_Speed(double Calculated_PID)
{
   //LImit Calculated PID
  current_speed = (int) current_speed + Calculated_PID;

  if(current_speed >= 255)
  {
    current_speed = 255;
  }
  else if (current_speed < 0)
    current_speed = 0;

  //Write pwm to PWM pin
  analogWrite(Pin._PWM_Pin,current_speed);


//    Serial.print("Current: ");
//    Serial.println(current_speed);
//    Serial.print("Position: ");
//    Serial.println(encoder.Position);


}

void Motor::Compute_PID(PID myPID)
{

  //Limit_Speed(encoder.Velocity,encoder.Pre_Velocity);
  //Serial.print(encoder.Velocity);Serial.print("\t");

  input = abs(encoder.Velocity);
  myPID.Compute();
  //Serial.println(current_speed);
}
double Motor::Speed(double left, double right)
{
  double velocity = 0;
  if(_motor)
  {
    //velocity = ( 2*linear*100 + LENGTH_OF_TWO_WHEELS * angular) / (2);//*WHEEL_RADIUS);
      velocity = right*100;
  }
  else
  {
//    velocity = ( 2*linear*100 - LENGTH_OF_TWO_WHEELS * angular) / (2);//*WHEEL_RADIUS);
      velocity = left*100;
  }
  //Can limit the speed of robot when the robot goes backward ( maybe 1/2 forward speed)
  Direction(velocity);

  //Calculate ticks per rev
  double temp_tick_per_rev = Convert_Speed_To_TickPerSecond(abs(velocity));
  //Limit the speed of the robot
  if (temp_tick_per_rev > MAX_Tick_Per_Rev)
    setpoint = MAX_Tick_Per_Rev;
  else
    setpoint = temp_tick_per_rev;

}
double Motor::Convert_Speed_To_TickPerSecond(double _speed)
{
  double ticks = (double) _speed / WHEEL_RADIUS;        //Rad/s
  //Convert to revolution per second
  ticks = (double) ticks / (2*PI);          //Rev/s
  //Tick per Second
  ticks = (double) ticks * TICK_PER_REVOLUTION;     // Per second

  ticks = (double) ticks * TIME_FRAME / ONE_SECOND;      // TIME_FRAME in us => Multi with 1000 to convert to micro second

  //Return ticks per Time frame
  return ticks;
}
void Motor::Direction(double velocity)
{
  //Identify direction of the robot when receiving user's command
  if(velocity > 0)
  {
     digitalWrite(Pin._MotorEnable_1,HIGH);
     digitalWrite(Pin._MotorEnable_2,LOW);
  }
  else
  {
    digitalWrite(Pin._MotorEnable_1,LOW);
    digitalWrite(Pin._MotorEnable_2,HIGH);
  }
}
void Motor::SetUpPID(PID &myPID)
{
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  
}


