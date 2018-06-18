#include "Motor.h"
#include "Variables.h"
Motor motor_right;
Motor motor_left;
PID rightPID(&motor_right.input,&motor_right.output,&motor_right.setpoint,KP,KD,KI,DIRECT);
PID leftPID(&motor_left.input,&motor_left.output,&motor_left.setpoint,KP,KD,KI,DIRECT);


String CommandOfTablet = "";
String linear = "";
String angular = "";
int Received_Command;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  
 TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-255, 255);  
   // motor_right.SetUpPID(rightPID);
  //motor_left.SetUpPID(leftPID);  

  
  motor_right.Begin(encodPinA1, encodPinB1, Mo_A1, Mo_A2, PWM_Control_Mo_A,Motor_Right);
  //motor_left.Begin(encodPinA2,encodPinB2,Mo_B1 ,Mo_B2,PWM_Control_Mo_B,Motor_Left);
  attachInterrupt(digitalPinToInterrupt(encodPinA1), isr, RISING);
  //attachInterrupt(digitalPinToInterrupt(3), isrB, RISING);
  Timer1.initialize( TIME_FRAME);
  Timer1.attachInterrupt( timer_interrupt);

  Serial.println("Ready");
  
}
void loop() {
  // put your main code here, to run repeatedly:
  
  CommandOfTablet = "";
  while(Serial.available())
  {
    Received_Command = Serial.read();
    //Store command to temp variable
    CommandOfTablet +=(char) Received_Command;
    delay(10);
  }
  if(CommandOfTablet.length())
  {
    int _beg = CommandOfTablet.indexOf("l");
    int _end = CommandOfTablet.indexOf("a");
    if( _beg != -1 && _end != -1)
    {
       linear = CommandOfTablet.substring(_beg + 1, _end);
       angular = CommandOfTablet.substring(_end + 1, CommandOfTablet.length());
    }
   Serial.print("linear: ");
   Serial.print(linear);
   Serial.print("angular: ");
   Serial.println(angular);
   motor_right.Speed(linear.toInt(),angular.toInt());
  }
  
  motor_right.Compute_PID(rightPID);
   //motor_left.Compute_PID(leftPID);
  
}
void isr()
{
  motor_right.Interrupt();
}
void isrB()
{
  motor_left.Interrupt();
}
void timer_interrupt()
{
  motor_right.Calculate_Speed();
  //motor_left.Calculate_Speed();
}

