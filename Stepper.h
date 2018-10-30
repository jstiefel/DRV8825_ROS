//============================================================================
// Name        : Stepper.h
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 11.05.2018
// Copyright   : BSD 3-Clause
// Description : Control of a Stepper Motor (Nema 14) with an Arduino and
//               DRV8825 or similar motor driver.
//============================================================================

#pragma once
#include <AccelStepper.h>

#ifndef STEPPER_H_
#define STEPPER_H_

//Define pins
#define M0 11
#define M1 12
#define M2 13
#define home_switch 9
#define home_switch_power 7

namespace asc_node {

/*!
   Class containing the control of the Stepper Motor over Arduino
*/
class Stepper {
  public:
    /*!
       Constructor
    */
    Stepper(int step_pin, int dir_pin);

    /*!
       Destructor
    */
    virtual ~Stepper();

    void moveRel(const float targetRelPos);
    void moveAbs(const float targetAbsPos);
    void stop();
    void currentState(float* position);
    void run();
    void homing();


  private:

    AccelStepper stepper; // pin 2 = step, pin 5 = direction
    void set_params();
    void set_homing_params();
    bool homing_completed;
    int microsteps;
    int i_trans;
    void set_microsteps(int microsteps);
    int degToSteps(float deg);
    float stepsToDeg(int steps);


};

#endif /* STEPPER_H_ */

} /* namespace */


