/*
   Robot.ino

    Created on: Jul 20, 2018
        Author: Thanh Hoang
*/

#include "Variables.h"
#include "TimerOne.h"
#include "DifferentialDrive.h"

DDrive d_drive;
int stage = 0;

void setup() {
  // establish Serial communication channel
  Serial.begin(9600);
  
  // command rx LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // set 31KHz PWM to prevent motor noise
  TCCR1B = TCCR1B & 0b11111000 | 1;

  // setup pins for each motors
  d_drive.SetRightMotorPins(encodPinA1, encodPinB1, Mo_A1, Mo_A2, PWM_Control_Mo_A);
  d_drive.SetLeftMotorPins(encodPinA2, encodPinB2, Mo_B1 , Mo_B2, PWM_Control_Mo_B);

  attachInterrupt(digitalPinToInterrupt(encodPinA1), RightMotorInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encodPinA2), LeftMotorInterrupt, RISING);

  // Timer1 for increase motor's speed smoothly
  Timer1.initialize(TIMER_INTERVAL);
  Timer1.attachInterrupt(TimerInterrupt);
}

void loop() {
  RxCommand();
}

// Interrupt for measuring right motor's velocity
void RightMotorInterrupt() {
  d_drive.RightMotorInterrupt();
}

// Interrupt for measuring left motor's velocity
void LeftMotorInterrupt() {
  d_drive.LeftMotorInterrupt();
}

// Timer interrupt function
int time_counter = 0, flag = 0, code = 0x67;

void TimerInterrupt() {

  d_drive.TimerInterrupt();
  
  if (DEBUG) { // debug mode. toggle speed every 3s
    d_drive.ParseCommand(code);
    if (time_counter++ > 3 * ONE_SECOND / TIMER_INTERVAL) {
      code = (!flag)?0xA7:0x67;
      flag ^= 1;
      time_counter = 0;
    }
  }
}

void RxCommand() {
  unsigned char rx = 0x67;
  while (Serial.available()==0){
    Serial.read();
    delay(1);
  }
  rx = Serial.read();
  stage ^= 1;
  digitalWrite(LED_BUILTIN, stage);
  d_drive.ParseCommand(rx);
}

