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
  universal_msgs
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs universal_msgs message_runtime)

include_directories(
  /usr/include/eigen3
  include
  ${catkin_INCLUDE_DIRS}
)


#self-defined library
add_library(config src/libs/config.cpp)
add_library(eyeHand src/libs/eyeHand.cc)

######-------------------add_executable--------------------
#add_executable(image_collector src/image_collector.cc)
#target_link_libraries(image_collector ${catkin_LIBRARIES} ${OpenCV_LIBS} config)

add_executable(image_collector_jaka src/image_collector_jaka.cc)
target_link_libraries(image_collector_jaka ${catkin_LIBRARIES} ${OpenCV_LIBS} config)

add_executable(RGBD_collector_jaka src/RGBD_collector_jaka.cc)
target_link_libraries(RGBD_collector_jaka ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} config)

add_executable(image_collector_guided src/image_collector_guided.cc)
target_link_libraries(image_collector_guided ${catkin_LIBRARIES} ${OpenCV_LIBS} config)

add_executable(calibration src/calibration.cc)
target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBS} eyeHand config)

add_executable(cross src/cross.cc)
target_link_libraries(cross ${catkin_LIBRARIES} ${OpenCV_LIBS} config)


