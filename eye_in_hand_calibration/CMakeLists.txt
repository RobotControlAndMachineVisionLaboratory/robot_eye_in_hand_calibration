cmake_minimum_required(VERSION 2.8.3)
project(eye_in_hand_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
set( CMAKE_CXX_COMPILER "/usr/bin/clang++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  ur_msgs
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files(
  FILES
  Command.msg
  RobotMsg.msg
)

generate_messages(DEPENDENCIES sensor_msgs std_msgs ur_msgs)
catkin_package(CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#self-defined library
add_library(config src/libs/config.cc)
add_library(eyeHand src/libs/eyeHand.cc)

######-------------------add_executable--------------------
#add_executable(image_collector src/image_collector.cc)
#add_dependencies(image_collector eye_hand_calibration_generate_message_cpp)
#target_link_libraries(image_collector ${catkin_LIBRARIES} ${OpenCV_LIBS} config)

#add_executable(image_collector_jaka src/image_collector_jaka.cc)
#add_dependencies(image_collector_jaka eye_hand_calibration_generate_message_cpp)
#target_link_libraries(image_collector_jaka ${catkin_LIBRARIES} ${OpenCV_LIBS} config)


add_executable(calibration src/calibration.cc)
target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBS} eyeHand config)

#add_executable(cross src/cross.cc)
#target_link_libraries(cross ${catkin_LIBRARIES} ${OpenCV_LIBS} config)
