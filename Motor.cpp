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
	cmd = {0,0};

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
  //setpoint = MAX_Tick_Per_Rev;
}


void Motor::TickPerRevolution()
{

  MAX_Tick_Per_Rev = Convert_Speed_To_TickPerSecond(MAX_SPEED);

  //Limit Acceleration
  Acc_Limit = Convert_Speed_To_TickPerSecond(ACC_MAX);
}

void Motor::Limit_Speed(double Final_Vel, double Init_Vel)
{
  //Invert Acc_Limit if the TR goes backward
  if(Final_Vel - Init_Vel > Acc_Limit)
        encoder.Velocity = Acc_Limit;//Init_Vel + Acc_Limit;

}

void Motor::Interrupt()
{
    if (digitalRead(Pin._encodB) == HIGH)             encoder.Position++;           // if (digitalRead(encodPinB1)==HIGH)  count ++; (if PINB1 &0b00000001)
    else                                          	  encoder.Position--;             // if (digitalRead(encodPinB1)==LOW)   count --;
}


void Motor::Write_PWM(double Calculated_PID)
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

}

void Motor::Compute_PID(PID myPID)
{
  //Limit_Speed(encoder.Velocity,encoder.Pre_Velocity);
  //Serial.print(encoder.Velocity);Serial.print("\t");
  input = encoder.Velocity;
  myPID.Compute();
  //Serial.println(current_speed);
}
double Motor::Speed(double velocity)
{
  //Can limit the speed of robot when the robot goes backward ( maybe 1/2 forward speed)
  Direction(velocity);

  //Calculate ticks per rev
  double temp_tick_per_rev = Convert_Speed_To_TickPerSecond(abs(velocity));
  //Limit the speed of the robot
  if (temp_tick_per_rev > MAX_Tick_Per_Rev)
    setpoint = MAX_Tick_Per_Rev;
  else
    setpoint = temp_tick_per_rev;
  if(DEBUG) Serial.println(setpoint);
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
  else if (velocity < 0)
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
bool Motor::bitLocation(int Rx_Command, byte bit_number)
{
  return (bitRead(Rx_Command,bit_number));
}

void Motor::GoStraight(int Rx_Command)
{
  if(bitLocation(Rx_Command,FORWARD_BIT))
  {
     if(bitLocation(Rx_Command,RIGHT_BIT))
     {
      if(DEBUG)
      {
        Serial.println("Keep Straight Right");
      }
      DifferentialDrive(local_speed,-15);//SPEED*0.7 ,-20);

      return;
     }
     
     if(bitLocation(Rx_Command,LEFT_BIT))
     {
       if(DEBUG)
      {
        Serial.println("Keep Straight Left");
      }
      //Serial.println("Keep Straight Left");
      DifferentialDrive(local_speed,15);//SPEED*0.7,20);
      
      return;
    }
    
    if(DEBUG)
      {
        Serial.println("Keep Straight");
      }
    DifferentialDrive(local_speed,0);
    return;
  }
}
void Motor::GoBack(int Rx_Command)
{
  //Serial.println("Go back");
  if(bitLocation(Rx_Command,BACKWARD_BIT))
  {
     
     if(bitLocation(Rx_Command,RIGHT_BIT))
     {
       if(DEBUG)
      {
        Serial.println("Keep Back Right");
      }
      DifferentialDrive(local_speed * BACKWARD_LIMIT,-15);
      return;
     }
     if(bitLocation(Rx_Command,LEFT_BIT))
     {
       if(DEBUG)
      {
        Serial.println("Keep Back Left");
      }
      DifferentialDrive(local_speed * BACKWARD_LIMIT,15);
      return;
     }
      if(DEBUG)
      {
        Serial.println("Go Back");
      }
      DifferentialDrive((-1)*local_speed,0);//SPEED * BACKWARD_LIMIT,0);
     return;
  }
  if(!(bitLocation(Rx_Command,BACKWARD_BIT) || bitLocation(Rx_Command,FORWARD_BIT)))
  {
    
    TurnRight(Rx_Command);
    TurnLeft(Rx_Command);
  }
}
void Motor::TurnRight(int Rx_Command)
{
  if(bitLocation(Rx_Command,RIGHT_BIT))
  {
     if(DEBUG)
      {
        Serial.println("Turn Right");
      }
   //DifferentialDrive(SPEED,-90);
   if(_motor)
   {
    DifferentialDrive(0,0);
   }
   else
   {
    DifferentialDrive(local_speed,0);//SPEED/3,0);
   }
  }
}
void Motor::TurnLeft(int Rx_Command)
{
  if(bitLocation(Rx_Command,LEFT_BIT))
  {
     if(DEBUG)
      {
        Serial.println("Turn Left");
      }
    //DifferentialDrive(SPEED,90);
   if(_motor)
   {
    DifferentialDrive(local_speed,0);//SPEED/3,0);
   }
   else
   {
    DifferentialDrive(0,0);
   }
  }
}
void Motor::Movement(int received_command)
{
   if(received_command == 0)
     {
        cmd.Counter= 0;
        cmd.PreComm = received_command ;
        Write_PWM(-255);
        Speed(STOPPING);
        return;
     }
   //Check if number or not
   if(!isDigit(received_command)) 
   {
    if(received_command == (int) '\n') received_command = 10;
    else
      return;
   }
   if(cmd.PreComm == received_command)
   {
      cmd.Counter += 1;
      if(cmd.Counter >= COMMANDCOUNTER)
      {
        //Serial.println("Set new setpoint");
        //setpoint = 100;
        local_speed += 5;
      }
   }
   else
   {
    local_speed = 10;
    cmd.Counter = 0;
    cmd.PreComm = received_command ;
    
   }
   GoStraight(received_command);
   GoBack(received_command);

   
}
void Motor::DifferentialDrive(double velocity,int degree)
{
  double calculated_velocity = 0;
  double ang_velocity = (double) PI * degree / 180;
  //Maybe divide time frame
  if(_motor)
  {
    calculated_velocity = (double) (2 * velocity + ang_velocity * LENGTH_OF_TWO_WHEELS) / (2);
    if(DEBUG)
      {
        Serial.print("Motor Right: ");
        Serial.println(calculated_velocity);
      }
  }
  else
  {
    calculated_velocity = (double) (2 * velocity - ang_velocity * LENGTH_OF_TWO_WHEELS) / (2);
    //Serial.println(calculated_velocity);
    if(DEBUG)
      {
        Serial.print("Motor Left: ");
        Serial.println(calculated_velocity);
      }
  }
  Speed(calculated_velocity);
}
void Motor::SetOutputToPWM(PID myPID)
{

  encoder.Velocity = (double) abs(encoder.Position);   //in us
  Limit_Speed(encoder.Velocity,encoder.Pre_Velocity);
  encoder.Pre_Velocity = encoder.Velocity;
  Compute_PID(myPID);
  //Set the PWM
  Write_PWM(output);

  //Reset Position
  encoder.Position = 0;
}




