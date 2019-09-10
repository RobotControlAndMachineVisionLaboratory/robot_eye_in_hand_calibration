cmake_minimum_required(VERSION 2.8.3)
project(universal_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  ur_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Command.msg
  RobotMsg.msg
)

## Generate services in the 'srv' folder
#add_service_files(
 # FILES
 # servise1.srv
 # service2.srv
#)

## Generate actions in the 'action' folder
#add_action_files(
 # FILES
 # Action1.action
 # Action2.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs ur_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)
