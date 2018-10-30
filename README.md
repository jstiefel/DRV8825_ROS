# asc_node - Arduino Stepper Controller ROS Node
This firmware uses an Arduino Uno Rev. 3 as a ROS node to control a stepper motor by using a common motor driver. Communication to a client (PC) is established by using rosserial_arduino package. Rosserial provides a ROS communication protocol that works over Arduino's UART. 
The node runs on the Arduino independently of a client and publishes messages which can be read out over rosserial.

## Hardware
- Arduino Uno Rev. 3
- DRV8825 motor driver

## Why DRV8825
Adafruit motor shield is very limited due to missing current limitation and voltage restriction. DRV8825 motor driver can be used over a wider range. Node works with all generic two-pin (DIR/STEP) motor drivers.

## Functions
The node accesses functions which are used to control a stepper motor. 

## Set up Arduino and install rosserial 
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

## Arduino IDE
Arduino IDE is used. Project is divided into several class files. Need to install ros_lib first. 

## ros_lib and custom dependencies
To use custom dependencies (e.g. custom messages, services etc.), you have to build these first (catkin build) and then install ros_lib by using the following commands (and not over Arduino IDE):

`rm -r ~/Arduino/libraries/ros_lib`

`rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries`

## Access data from client console
`roscore`

`rosrun rosserial_python serial_node.py /dev/ttyUSB0`
(rosserial client application)

`rostopic echo <topic_name>`

`rostopic pub <topic_name> std_msgs/Int32 --once "<value>"`

## List of Topics/Services
Homing:

`rosservice call /stepper_homing_srv`

Set position (between -44 and 50 degree, only possible after homing):

`rosservice call /stepper_abs_pos_srv "target_abs_position: 0.0"`

Joint state information (only position):

`rostopic echo /joint_state`

## rosserial
http://wiki.ros.org/rosserial

http://wiki.ros.org/rosserial_arduino/Tutorials

## ROS package dependencies

Only to create messages. Replace by your own:

- custom_communication

## Arduino Libraries
- AccelStepper

## Sources
https://www.pololu.com/product/2133

http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

## Attention
- Stepper doesn't stop exactly at the homing switch.
- Problems with serial connection arise if the delays are set too short.
- Buffer size and number of subscribers/publishers was limited due to low dynamic memory (in ros.h)

