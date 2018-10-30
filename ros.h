#ifndef _ROS_H_
#define _ROS_H_

#include "ArduinoHardware.h"
#include "ros/node_handle.h"


namespace ros
{
  //default is 25, 25, 512, 512
  typedef NodeHandle_<ArduinoHardware, 3, 3, 128, 128> NodeHandle; // default 25, 25, 512, 512
}

#endif
