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
BUS bus = {0,true,false};
int change_state_counter = 0;
void setup() {
  Serial.begin(9600);
  if(DEBUG) Serial.begin(9600);

 
 TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  //Set up PID 
 
  motor_right.SetUpPID(rightPID);
  motor_left.SetUpPID(leftPID);  
  
  motor_right.Begin(encodPinA1, encodPinB1, Mo_A1, Mo_A2, PWM_Control_Mo_A,Motor_Right);
  motor_left.Begin(encodPinA2,encodPinB2,Mo_B1 ,Mo_B2,PWM_Control_Mo_B,Motor_Left);
  attachInterrupt(digitalPinToInterrupt(encodPinA1), isr, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinA2), isrB, RISING);
  Timer1.initialize( TIME_FRAME);
  Timer1.attachInterrupt( timer_interrupt);
  if(DEBUG) Serial.println(motor_right.MAX_Tick_Per_Rev);
  if(DEBUG) Serial.println("Ready");
  motor_right.Speed(STOPPING);
  motor_left.Speed(STOPPING);
//*/
  Serial.println("Ready");
  if(DEBUG) Serial.println("Ready");

  bus.counter = ONE_SECOND/TIME_FRAME + 1;
  if(DEBUG) Serial.println(bus.counter);

}
void loop() {
  // put your main code here, to run repeatedly:
    
    Rx_Command();

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
  motor_right.SetOutputToPWM(rightPID);
  motor_left.SetOutputToPWM(leftPID);
  change_state_counter++;
  if(change_state_counter >= bus.counter)
  {
    change_state_counter = 0;
    if(bus.curr_state == bus.prev_state)
    {
      //Stop the motor for safety
      //Serial.println("Stop motor");
      motor_right.Movement(STOPPING);
      motor_left.Movement(STOPPING);
    }
    else
    {
      bus.curr_state = bus.prev_state;
      //Serial.println("Still good");
    }
  }
 //*/
}
void Rx_Command()
{
  int TempValue = 0;
  if(Serial.available())
  {
     CommandOfTablet = "";
     Received_Command = 0;
      while(Serial.available())
    {
      Received_Command = Serial.read();
    }
    TempValue = Received_Command;
    bus.prev_state = not bus.curr_state;
    if(DEBUG)
    {
    Serial.print("curr: ");
    Serial.print(bus.curr_state);
    Serial.print("  prev: ");
    Serial.println(bus.prev_state);
    Serial.println(TempValue);
    }
    motor_right.Movement(TempValue);
    motor_left.Movement(TempValue);

  }
 
}


