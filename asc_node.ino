//===============================================================================
// Name        : asc_node.ino
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 11.05.2018
// Copyright   : BSD 3-Clause
// Description : Main node for ROS Arduino Stepper Motor Control by using DRV8825.
//               Some functions are commented out to save memory.
//===============================================================================

#include "ros.h" // "" are used instead of < > to use custom ros.h (because of dynamic memory problems)
#include "Stepper.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <custom_communication/TargetAbsPos.h>
#include <custom_communication/TargetRelPos.h>

ros::NodeHandle nh;
asc_node::Stepper Stepper(2, 5); //step_pin=2, dir_pin=5
float position;

void homing(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  Stepper.homing();
  res.success = 1;
  /*Check if success message is sent after homing completed, then "success" can be used to control flow.*/
}

void moveAbsCallback(const custom_communication::TargetAbsPos::Request& req, custom_communication::TargetAbsPos::Response& res) {
  /*this is only a helper function to reroute to real callback function*/
  Stepper.moveAbs(-req.target_abs_position);
}

void moveRelCallback(const custom_communication::TargetRelPos::Request& req, custom_communication::TargetRelPos::Response& res) {
  /*this is only a helper function to reroute to real callback function*/
  Stepper.moveRel(-req.target_rel_position);
}

std_msgs::Float32 msg;
  //add class callback function if issue solved instead of using helper callback fcts. - see first commit
  ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> homing_server("stepper_homing_srv", &homing);
  ros::ServiceServer<custom_communication::TargetAbsPos::Request, custom_communication::TargetAbsPos::Response> move_abs_server("stepper_abs_pos_srv", &moveAbsCallback);
//  ros::ServiceServer<custom_communication::TargetRelPos::Request, custom_communication::TargetRelPos::Response> move_rel_server("rel_pos_srv", &moveRelCallback);
  ros::Publisher jointState_publisher("joint_state", &msg);

void setup() {
  // put your setup code here, to run once:
  nh.initNode(); //other initialization in rosserial than normally
  nh.advertiseService(homing_server);
//  nh.advertiseService(move_rel_server);
  nh.advertiseService(move_abs_server);
  nh.advertise(jointState_publisher);
}

void loop() {
  // put your main code here, to run repeatedly:
  //get data for message here
  Stepper.currentState(&position);
  msg.data = -position;
  jointState_publisher.publish(&msg);
  nh.spinOnce();
  Stepper.run(); //this makes in not-blocking, because run is executed at each loop and positions can be changed between
  delay(10); //because of rosserial, more delay leads to slower motion, less delay to problems with serial connection and calculations
}
