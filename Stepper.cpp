//============================================================================
// Name        : Stepper.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 11.05.2018
// Copyright   : BSD 3-Clause
// Description : Control of a Stepper Motor (Nema 14) with an Arduino and
//               DRV8825 or similar motor driver.
//============================================================================

#include "Stepper.h"

namespace asc_node {

Stepper::Stepper(int step_pin, int dir_pin)
  : stepper(AccelStepper::DRIVER, step_pin, dir_pin)
{
  homing_completed = 1;
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(home_switch_power, OUTPUT);
  digitalWrite(home_switch_power, HIGH);
  pinMode(home_switch, INPUT);
 
  set_params();
  microsteps = 16; //1,2,4,16,32
  set_microsteps(microsteps); //This functions sets microsteps on hardware according to value 1, 2, 4, 16, 32
  i_trans = 60 / 12; //transmission belt

  stepper.stop(); //stop all actions before starting (only to be on the safe side)
}

Stepper::~Stepper()
{

}

void Stepper::set_homing_params()
{
  stepper.setMaxSpeed(50);
  stepper.setAcceleration(20);
}

void Stepper::set_params()
{
  stepper.setMaxSpeed(200); //default: 100
  stepper.setAcceleration(60); //default: 40
}

void Stepper::set_microsteps(int microsteps)
{
  /* Arduino connection:
      pin 11 = M0
      pin 12 = M1
      pin 13 = M2
  */
  switch (microsteps) {
    case 1:
      {
        digitalWrite(M0, LOW);
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        break;
      }
    case 2:
      {
        digitalWrite(M0, HIGH);
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        break;
      }
    case 4:
      {
        digitalWrite(M0, LOW);
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        break;
      }
    case 8:
      {
        digitalWrite(M0, HIGH);
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        break;
      }
    case 16:
      {
        digitalWrite(M0, LOW);
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        break;
      }
    case 32:
      {
        digitalWrite(M0, HIGH);
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        break;
      }
  }
}

void Stepper::homing()
{
  /*This function is blocking because it runs in a while loop, until homing is completed. This is useful in this case.
     Based on: https://www.brainy-bits.com/setting-stepper-motors-home-position-using-accelstepper/
  */
  set_homing_params();
  stepper.setCurrentPosition(0);
  
  long initial_homing = -1;
  while (digitalRead(home_switch)) {
    stepper.moveTo(initial_homing);
    initial_homing--;
    stepper.run();
    delay(5);
  }
  stepper.stop();

  //tape blocks homing switch at the moment for a while... therefore, this part is commented out. Also if homing can not be executed without any errors, check if homing switch is blocked
//  stepper.setCurrentPosition(0);
//  //move away:
//  set_homing_params();
//  initial_homing = 1;
//
//  while (!digitalRead(home_switch)) {
//    stepper.moveTo(initial_homing);
//    initial_homing++;
//    stepper.run();
//    delay(5);
//  }
//  stepper.stop();

  //set beta offset to -44deg with foam on homing switch:
  stepper.setCurrentPosition(-degToSteps(44));
  
  set_params();

  homing_completed = 0;
}

int Stepper::degToSteps(float deg)
{
  int steps;
  steps = microsteps * i_trans * deg / 1.8;
  return steps;
}

float Stepper::stepsToDeg(int steps)
{
  float deg;
  deg = (steps * 1.8) / (microsteps * i_trans);
  return deg;
}

void Stepper::moveRel(const float targetRelPos)
{
  //targetRelPos in degrees of end-effector
  //calculate it to steps
  int targetRelSteps;
  if (homing_completed == 0 && (targetRelPos + stepsToDeg(stepper.currentPosition())) >= -44 &&  (targetRelPos + stepsToDeg(stepper.currentPosition())) <= 50)
  {
    targetRelSteps = degToSteps(targetRelPos);
    stepper.move(targetRelSteps);
  }
}

void Stepper::moveAbs(const float targetAbsPos)
{
  //targetAbsPos in degrees of end-effector
  //calculate it to steps
  //check if homing was done first
  //check if abs position is in possible range
  
  int targetAbsSteps;
  if (homing_completed == 0 && targetAbsPos >= -44 && targetAbsPos <= 50)
  {
    targetAbsSteps = degToSteps(targetAbsPos);
    stepper.moveTo(targetAbsSteps); //because CCW direction is positive in our case
  }
}

void Stepper::run()
{
  /*Function which has to be called regularly in loop*/
  stepper.run();
}

void Stepper::stop()
{
  stepper.stop();
}

void Stepper::currentState(float* position)
{
  *position = stepsToDeg(stepper.currentPosition());
}

} /* namespace */

