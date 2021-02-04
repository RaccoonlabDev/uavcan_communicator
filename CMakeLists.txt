cmake_minimum_required(VERSION 3.0.2)
project(drone_communicators)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

find_library(UAVCAN_LIB uavcan REQUIRED)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -std=c++11")

add_message_files(
  FILES
  RawAirData.msg
  StaticPressure.msg
  StaticTemperature.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)