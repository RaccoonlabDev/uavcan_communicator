cmake_minimum_required(VERSION 3.0.2)
project(uavcan_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES

  RawAirData.msg
  StaticPressure.msg
  StaticTemperature.msg
  Fix.msg
  EscStatus.msg
  CircuitStatus.msg
  IceReciprocatingStatus.msg
  IceFuelTankStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
