#include "Motor.h"
#include "Variables.h"
#include "WString.h"
Motor motor_right;
Motor motor_left;
PID rightPID(&motor_right.input,&motor_right.output,&motor_right.setpoint,KP,KD,KI,DIRECT);
PID leftPID(&motor_left.input,&motor_left.output,&motor_left.setpoint,KP,KD,KI,DIRECT);

String CommandOfTablet = "";
String left = "";
String right = "";
char Received_Command;
void setup() {
  
  Serial.begin(9600);
  
 TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  //Set up PID 
 
  motor_right.SetUpPID(rightPID);
  //motor_left.SetUpPID(leftPID);  
  
  motor_right.Begin(encodPinA1, encodPinB1, Mo_A1, Mo_A2, PWM_Control_Mo_A,Motor_Right);
  //motor_left.Begin(encodPinA2,encodPinB2,Mo_B1 ,Mo_B2,PWM_Control_Mo_B,Motor_Left);
  attachInterrupt(digitalPinToInterrupt(encodPinA1), isr, RISING);
  //attachInterrupt(digitalPinToInterrupt(3), isrB, RISING);
  Timer1.initialize( TIME_FRAME);
  Timer1.attachInterrupt( timer_interrupt);
  Serial.println(motor_right.MAX_Tick_Per_Rev);
  Serial.println("Ready");
  motor_right.Speed(0,0);
//*/
}
float test = 0;
void loop() {
  // put your main code here, to run repeatedly:



//motor_right.Speed(2.2,2.2);
  Read_Command();
  
  motor_right.Compute_PID(rightPID);
   //motor_left.Compute_PID(leftPID);
  //*/
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
void Read_Command()
{
  if(Serial.available()>0)
  {
    while(Serial.available())
    {
      Received_Command = Serial.read();
      if(Received_Command=='e')
      {
        Serial.println("break");
        break;
      }
      Serial.println((int)Received_Command);
      CommandOfTablet += Received_Command;
      
      delay(1);
      
    }
  
    if(CommandOfTablet.length())
    {
      int _beg = CommandOfTablet.indexOf("l");
      int _end = CommandOfTablet.indexOf("r");
    
      if( _beg != -1 && _end != -1)
      {
        left = CommandOfTablet.substring(_beg + 1, _end);
        right = CommandOfTablet.substring(_end + 1, CommandOfTablet.length());
      }
       
      motor_right.Speed(left.toDouble(),right.toDouble());
    
    }

    CommandOfTablet = "";
    //Serial.print(left);Serial.print("\t");
    //Serial.println(right);
  }
  
}

